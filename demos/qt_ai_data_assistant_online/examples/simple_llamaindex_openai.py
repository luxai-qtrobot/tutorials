# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from utils.logger import Logger
from llm.llm_llamaindex_openai import LLMLamaIndexOpenAI
from vector_store.vector_store_astradb import VectorStoreAstraDB
from data_loader.local_files_reader import LocalFilesReader 



SYSTEM_PROMOT = '''     
    You are a humanoid social robot assistant named "QTrobot". 
    You are designed to support various use cases, including the education of children with autism and other special needs, as well as human-robot interaction research and teaching.    

    Follow these guidelines when answering questions:
    - Do not include the user ID in your response-just answer the query itself.
    - Always respond in one or two brief sentences. Keep your sentences as short as possible.
    - Respond in plain text without using EOL ("\n") or any other text formatting.
    - Do not use bullet points or formulas in your response. Instead, provide a brief summary. 
    - Do not respond with emoticon.
    - If you need to provide a long list that includes more than three sentences, provide a few items and ask if the user would like to hear more.
    - Avoid mentioning the name of the file, document in your response.
    - Do not say "According to the information", "According to the context" or similar phrases. Instead, only provide the response.
    - Your priority is to keep responses short, clear, and to the point. Keep your sentences very short.    
 '''


if __name__ == '__main__':
    
    current_dir = os.path.dirname(os.path.abspath(__file__))   
    default_documents_dir = os.path.abspath(os.path.join(current_dir, '..', 'documents'))

    # from where to load the documents
    document_loader = LocalFilesReader(
        document_path=default_documents_dir,
        document_formats=['.pdf'],
        max_num_documents=5
    )

    # where to store the vector embeddings
    vector_store_engine = VectorStoreAstraDB(
        token=os.environ.get('ASTRA_DB_TOKEN'),
        endpoint=os.environ.get('ASTRA_DB_ENDPOINT'),
        collection_name="my_collection_openai",
        embedding_dimension=1536 # default for text-embedding-3-small used in LLMLamaIndexOpenAI
    )

    chat = LLMLamaIndexOpenAI(
        api_key=os.environ.get('OPENAI_API_KEY'),
        model='gpt-4o-mini', 
        system_role=SYSTEM_PROMOT,
        data_loader=document_loader,
        vector_store_engine=vector_store_engine,  
        reload_documents=True)
    
    try:
        while True:
            user = input("User: ").strip()
            if not user:
                continue

            if user.lower() == 'exit':
                break        
            
            if user.lower() == 'forget_chat':
                chat.clear_memmory()
                continue
            
            if user.lower() == 'print_history':                                
                Logger.info(f"Memory length: {len(chat.memory.get_all())}, tokens: {chat.memory._token_count_for_messages(chat.memory.get_all())}\n")
                Logger.info(f"{chat.memory.json()}\n")
                continue

            t = time.time()
            for msg in  chat.get_stream_response(text=user):            
                lat = int((time.time() - t)*1000)                
                Logger.info(f"({lat}ms) {msg}")

    except Exception as e:
        Logger.error(e)        
       
    
    chat.close()
