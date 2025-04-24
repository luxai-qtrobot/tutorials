
from abc import ABC, abstractmethod
from llama_index.core.readers.base import BaseReader

class DataLoaderBase(ABC):
    
    @abstractmethod
    def get_documents(self) -> BaseReader:
        raise NotImplementedError("get_documents() must be implemented in subclasses")
