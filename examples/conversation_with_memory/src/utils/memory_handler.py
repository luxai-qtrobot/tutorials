from langchain_community.document_loaders import DirectoryLoader
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain_community.embeddings import HuggingFaceEmbeddings    
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.schema import Document
from openai import OpenAI
import json
import os
import re
import rospy
import torch


class MemoryHandler:
    def __init__(self):
        self.knowledge = rospy.get_param("/cwm/embeddingsmodel/knowledge_location", "data-qtrobot")# 1) specify the source of data
        self.vectorstorename= rospy.get_param("/cwm/embeddingsmodel/vectorstorename", "testindex")# 1) specify the source of data
        self.memory = "./memory-qtrobot"
        openaikey= rospy.get_param("/cwm/chatengine/OPENAI_KEY", None)#open("key.txt", 'r').read().strip()
        self.client = OpenAI(api_key = openaikey)
        self.loader=DirectoryLoader
        self.chunk_size=rospy.get_param("/cwm/embeddingsmodel/chunk_size", 250)
        self.chunk_overlap= rospy.get_param("/cwm/embeddingsmodel/chunk_overlap", 5)
        self.embeddingsengine =rospy.get_param("/cwm/embeddingsmodel/embeddingsengine", "openai")
        if self.embeddingsengine == 'openai':
            self.embeddings_model = OpenAIEmbeddings(openai_api_key=openaikey)
        elif self.embeddingsengine == 'hugging':
            model_name = "sentence-transformers/all-MiniLM-L6-v2"
            device="cuda" if torch.cuda.is_available() else "cpu"
            model_kwargs ={'device':device}
            encode_kwargs ={'normalize_embeddings':False}
            hf=HuggingFaceEmbeddings(model_name=model_name, model_kwargs=model_kwargs, encode_kwargs=encode_kwargs)
            self.embeddings_model = hf
        else:
            print("The use of this model is not yet implemented, using openai ada 002 instead")
            self.embeddings_model = OpenAIEmbeddings(openai_api_key=openaikey)

# 2) load data for retriaval: use directory loader here
    def loadData(self, datadir):
        self.loader = DirectoryLoader(datadir)
        docs = self.loader.load()
        print(len(docs))
        return docs

# 3) chunking    
# here we split by the number of tokens, but we can also split by markdown or other values using other splitters
# tiktoken is helpful if we need to make sure
# that the chunk size does not exceed the allowed number of tokens for OpenAI
    def split_documents(self, loadeddata):
        text_splitter = RecursiveCharacterTextSplitter.from_tiktoken_encoder(
        chunk_size=250, chunk_overlap=8)
        transformed=text_splitter.split_documents(loadeddata)
        return transformed
    


# 4) embed: translate all transformed data to embeddings
# 5) store in a vector store
    def make_knowledge_embeddings(self, chunked_docs, vectorstore_name: str):
        db = FAISS.from_documents(chunked_docs, self.embeddings_model)
        db.save_local(vectorstore_name)#this is to store index locally
        print("New vectorstore created: ", vectorstore_name)
        return db


# indexing (from time to time) - not yet handled here
    #def renew_index(self):
        #return
    
# if local vector store already exists, retrieves context from local
        # otherwise creates a new vector store and searches for context
    def retrieve_context_from_store(self, query, vectorstore_obj) -> str:
        context=""
        docs = vectorstore_obj.similarity_search(query)
        context=docs[0].page_content
        return context
    
    def create_new_vectorstore(self, vectorstore_name):
        data=self.loadData(self.knowledge)
        chunked = self.split_documents(data)
        db = self.make_knowledge_embeddings(chunked, vectorstore_name)
        return db
    
# checks if local vector store with the given name already exists
    def check_local_knowledge(self, vectorstore_name: str):
        try:
            # this is to open the idex again
            db = FAISS.load_local(vectorstore_name, self.embeddings_model)
            print("Opening existing vectorstore:", vectorstore_name)
            return db
        except:
            FileNotFoundError
            print("local kowledge does not exist")
            return False
        
    def get_local_knowledge(self, vectorstore_name: str):
        try:
            # this is to open the idex again
            db = FAISS.load_local(vectorstore_name, self.embeddings_model)
            print("Opening existing vectorstore:", vectorstore_name)
            return db
        except:
            FileNotFoundError
            print("Local vectorstore does not exist. Creating from documents...")
            data=self.loadData(self.knowledge)
            chunked = self.split_documents(data)
            db = self.make_knowledge_embeddings(chunked, vectorstore_name)
            return db


# if local vector store already exists, retrieves context from local
        # otherwise creates a new vector store and searches for context
    def retrieve_context(self, query, vectorstore_name: str) -> str:
        print("the question is: ", query)
        db = self.check_local_knowledge(vectorstore_name)
        context="dummy context"
        if db != False:
            docs = db.similarity_search(query)
            context=docs[0].page_content
            print("retrieved context ", context)
        else:
            data=self.loadData(self.knowledge)
            chunked = self.split_documents(data)
            db = self.make_knowledge_embeddings(chunked, vectorstore_name)
            docs = db.similarity_search(query)
            context=docs[0].page_content
            print("retrieved context from new vectorstore", context)
        return context
    
    # 3) chunking of past conversations: 
    # transform each conversation to a summary 
    def summarize_conversation(self, filecontent:str):
        systemmessage="You are a helpful assistant. You summarise meeting transcripts into short descriptions. A user gives you a transcript, and you return a summary of it. Here is one transcript from a user: "
        summarised=self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            max_tokens=250,
            temperature=1,
            messages=[{"role":"system", "content":systemmessage},
                {"role": "user", "content": filecontent}]
        )
        chatmessage = summarised.choices[0].message
        reformulated=chatmessage.content
        #print(reformulated)
        return reformulated
    
    #needs testing
    def create_memory_from_talk(self, storagename:str):
        # check if storage with the given name is already there
        storage=self.check_local_knowledge(storagename)
        if storage!=False:
            print("Storage already exists: ", storagename)
            return storage
        else:
            print("Creating new storage: ", storagename)
            summarised=[]
            for filename in os.listdir(self.memory):
                f=self.memory+"/"+filename
                conversation=open(f, 'r').read()
                summary=self.summarize_conversation(conversation)
                summarised.append(Document(page_content=summary, metadata={"source": filename}))
            storage=self.make_knowledge_embeddings(summarised, storagename)
            return storage

    def recall_from_memoires(self, prompt,  memoires_vs):
        result=memoires_vs.similarity_search(prompt)
        memoire=result[0].page_content
        print("retrieved context from new vectorstore", memoire)
        return memoire
