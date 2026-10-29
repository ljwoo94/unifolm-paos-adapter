"""One model request at a time; chunk playback never waits for inference."""
from collections import deque
import threading
import time
import math
from bisect import bisect_right

from .chunk import base_goal_at

DEFAULT_REQUEST_AFTER_FRACTION=2/3


class ChunkExecution:
    def __init__(self, *, model_factory, observe, clock=time.monotonic,
                 request_after_fraction=DEFAULT_REQUEST_AFTER_FRACTION):
        if (type(request_after_fraction)not in (int,float)or
                not math.isfinite(request_after_fraction)or not 0<=request_after_fraction<=1):
            raise ValueError('request_after_fraction must be finite within [0,1]')
        self._factory = model_factory
        self._observe = observe
        self._clock = clock
        self._request_after_fraction=float(request_after_fraction)
        self._condition = threading.Condition()
        self._stopped = False
        self._paused = False
        self._error = None
        self._chunk = None
        self._model = None
        self._thread = None
        self._events = deque(maxlen=128)
        self._responses = 0

    def start(self):
        with self._condition:
            if self._thread is not None or self._stopped:
                raise RuntimeError('execution can be started only once')
            self._thread = threading.Thread(target=self._run, name='unifolm-inference', daemon=True)
            self._thread.start()

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def paused(self):
        with self._condition:
            return self._paused

    @property
    def error(self):
        with self._condition:
            return self._error

    def snapshot(self):
        with self._condition:
            return dict(paused=self._paused, error=self._error, responses=self._responses,
                        events=list(self._events),request_after_fraction=self._request_after_fraction)

    def latest_chunk(self):
        with self._condition:
            return self._chunk

    def current_sample(self):
        with self._condition:
            chunk=None if self._stopped else self._chunk
        if chunk is None:return None
        index=bisect_right(chunk.grid,max(0.,self._clock()-chunk.received_at))-1
        return (chunk,index)if index<chunk.horizon else None

    def current_goal(self):
        with self._condition:
            chunk = None if self._stopped else self._chunk
        if chunk is None:
            return None
        return base_goal_at(chunk, elapsed_s=max(0., self._clock() - chunk.received_at))

    def resume(self):
        with self._condition:
            if self._stopped or not self._paused:
                return False
            self._paused = False
            self._error = None
            self._condition.notify_all()
            return True

    def stop(self):
        with self._condition:
            self._stopped = True
            self._chunk = None
            model = self._model
            self._condition.notify_all()
        # Interrupt an in-flight receive; do not wait while holding playback's lock.
        if model is not None:
            self._close_model(model)

    def _close_model(self,model):
        try:
            model.close()
        except Exception as error:
            with self._condition:
                self._events.append(dict(kind="cleanup_error", error=str(error)[:512]))

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)

    def _wait_for_observation(self):
        # Observe only after playback has progressed. Delay the camera request,
        # rather than capturing early and merely delaying the model call.
        with self._condition:
            while True:
                self._condition.wait_for(lambda:self._stopped or not self._paused)
                if self._stopped:return False
                chunk=self._chunk
                if chunk is None:return True
                due=chunk.received_at+chunk.grid[-1]*self._request_after_fraction
                remaining=due-self._clock()
                if remaining<=0:return True
                # Explicit stop/resume wakes this wait; playback runs separately.
                self._condition.wait(timeout=remaining)

    def _run(self):
        model = None
        try:
            while self._wait_for_observation():
                try:
                    if model is None:
                        model = self._factory()
                        with self._condition:
                            if self._stopped:
                                return
                            self._model = model
                    started = self._clock()
                    observation = self._observe()
                    chunk = model.infer(observation)
                    with self._condition:
                        if self._stopped:
                            return
                        self._chunk = chunk
                        self._responses += 1
                        self._events.append(dict(kind='chunk', received_at=chunk.received_at,
                            observation_requested_at=started,
                            observation_to_receipt_s=chunk.received_at - started,
                            horizon=chunk.horizon, period_s=chunk.period_s,model=getattr(model,'last_timing',None)))
                except Exception as error:
                    with self._condition:
                        if self._stopped:
                            return
                        self._paused = True
                        self._error = f'{type(error).__name__}: {error}'[:512]
                        self._events.append(dict(kind='paused', at=self._clock(), error=self._error))
                        self._model = None
                    if model is not None:
                        self._close_model(model)
                    model = None
        finally:
            with self._condition:
                self._model = None
            if model is not None:
                self._close_model(model)
