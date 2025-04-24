from abc import ABC, abstractmethod
from threading import Event, Thread
from utils.logger import Logger

class BaseNode(ABC):
    """
    BaseNode class.
    """

    def __init__(self, name=None, paused=False, setup_kwargs={}):
        """
        Initializes the BaseNode, sets up events, and starts the thread.

        Args:
            name (str, optional): Name of the node. Defaults to the class name if not provided.
            paused (bool, optional): start the node in paused mode
        """
        self.name = name if name is not None else self.__class__.__name__
        self.terminate_event = Event()
        self.pause_event = Event()
        if not paused:
            self.pause_event.set()
        self.setup(**setup_kwargs)
        self.thread = Thread(target=self._run)
        self.thread.start()

    def setup(self):
        """Performs any necessary setup before the thread starts. Override in subclasses."""
        pass

    def cleanup(self):
        """Performs cleanup after the thread has been terminated. Override in subclasses."""
        pass

    def interrupt(self):
        """Performs actions after the thread has been interrupted by terminate. Override in subclasses."""
        pass

    @abstractmethod
    def process(self):
        """
        Defines the main processing task of the thread.
        Must be implemented by subclasses.
        """
        raise NotImplementedError


    def paused(self):
        return not self.pause_event.is_set()

    def terminating(self):
        return self.terminate_event.is_set()

    def pause(self):
        """Pauses the processing by setting the pause_event."""
        self.pause_event.clear()
        Logger.debug(f"{self.name} paused.")

    def resume(self):
        """Resuming the processing by clearing the pause_event."""
        self.pause_event.set()
        Logger.debug(f"{self.name} resumed.")

    def terminate(self, timeout=None):
        """
        Stops the processing by setting the terminate_event and unpausing if necessary.
        This allows the thread to exit cleanly.
        """
        self.terminate_event.set()
        self.pause_event.set()
        self.interrupt()
        self.thread.join(timeout=timeout)
        Logger.debug(f"{self.name} terminated.")

    def _run(self):
        """
        The main loop for the thread. Processes data while not terminated and pauses if pause_event is set.
        Cleans up resources when the loop is exited.
        """
        Logger.debug(f"{self.name} started.")
        while not self.terminate_event.is_set():
            # If the pause_event is set, wait until it is cleared.
            self.pause_event.wait()            
            if self.terminate_event.is_set():
                break
            
            self.process()

        self.cleanup()
        


    def __str__(self):
        """Returns a user-friendly string representation of the object."""
        return f"Node '{self.name}' (type: {self.__class__.__name__})"
