"""One model request at a time; chunk playback never waits for inference."""
from collections import deque
import threading
import time

from .chunk import base_goal_at


class ChunkExecution:
    def __init__(self, *, model_factory, observe, clock=time.monotonic):
        self._factory = model_factory
        self._observe = observe
        self._clock = clock
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
                        events=list(self._events))

    def latest_chunk(self):
        with self._condition:
            return self._chunk

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

    def _run(self):
        model = None
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._stopped or not self._paused)
                    if self._stopped:
                        return
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
                            observation_to_receipt_s=chunk.received_at - started,
                            horizon=chunk.horizon, period_s=chunk.period_s))
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
