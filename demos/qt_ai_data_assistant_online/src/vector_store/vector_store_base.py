
from abc import ABC, abstractmethod
from llama_index.core.vector_stores.types import BasePydanticVectorStore
from llama_index.core.indices.base import BaseIndex
from llama_index.core.readers.base import BaseReader
from llama_index.core import StorageContext


class VectorStoreBaseEngine(ABC):
    
    @abstractmethod
    def get_index(self, document: BaseReader = None) -> BaseIndex:
        raise NotImplementedError("get_vector_store_engine() must be implemented in subclasses")


    @abstractmethod
    def is_empty(self) -> bool:
        raise NotImplementedError("empty() must be implemented in subclasses")

    @abstractmethod
    def clear(self):
        raise NotImplementedError("clear() must be implemented in subclasses")


