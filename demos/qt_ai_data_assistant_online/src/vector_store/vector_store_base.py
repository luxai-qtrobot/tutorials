
from abc import ABC, abstractmethod
from llama_index.core.vector_stores.types import BasePydanticVectorStore


class VectorStoreBaseEngine(ABC):
    
    @abstractmethod
    def get_store(self) -> BasePydanticVectorStore:
        raise NotImplementedError("get_vector_store_engine() must be implemented in subclasses")
    
    @abstractmethod
    def is_empty(self) -> bool:
        raise NotImplementedError("empty() must be implemented in subclasses")

    @abstractmethod
    def clear(self):
        raise NotImplementedError("clear() must be implemented in subclasses")

