# Embodied conversation with memory

This example shows how retrieval-augmented generation (RAG) techniques can be used to provide a social agent with a consistent personality and long-term memory.

This example implements parts of the interaction as online and offline options:
- Speech recognition: online with google Speech or offline with Riva of Vosk;
- Language generation: online with OpenAI or offline with FastChat Vicuna1.5-13B
- RAG: online embeddings with OpenAI or offline embeddings with all-MiniLM-L6-v2 (sentence transformers)

You can combine online and offline components as you prefer if you are using QTrobot AI@Edge. Intel-based versions of QTrobot are better suitable for online language generation, but other options are still available. For instance, you can use Vosk for offline speech recognition, encode your knowldge on device using local embeddings and use cloud-based generative models to produce fluent language from the context that you retrieve offline. 

We recommend to install all dependencies in a virtual environment (say `.venv`). General dependencies for all combinations are:
- rospkg
- pyyaml
- openai (even if you plan to work with offline language generation, because FastChat implements the same interface).
- nltk
- text2emotion
- emoji==1.6.3


## Installation 

Put the `conversation_with_memory` project in `catkin_ws` by creating a symbolic link and build it: 
```bash
cd ~/catkin_ws/src
ln -s ~/robot/code/tutorials/examples/conversation_with_memory/ ./

cd ~/catkin_ws/
catkin_make
```

### Setup environment and install dependencies 
```bash
cd ~/catkin_ws/src/conversation_with_memory
python3 -m venv venv
source venv/bin/activate
pip install -r requirements-RDV2AI@Edge.txt  # use requirements-RDV2.txt for QT with Nuc PC 
```

### Setup Fastchat docker server (only on RDV2AI@Edge for using with Vicuna LLM)
Follow [build and run fastchat-server docker](docker/fastchat-server/README.md) guildeline to run Fastchat docker with `vicuna-13b-v1.5` model. 



## Configure speech recognition

In the project [config file](config/conversation_with_memory.yaml), you can choose among vosk, riva and gspeech options. Please choose only one, for example `asr: "riva" `.  These are the prerequisites for making ASR work:

### gspeech (Online)

LuxAI provides a wrapper for Google Speech. Follow instructions [here](https://github.com/luxai-qtrobot/software/tree/master/apps/qt_gspeech_app) to set up your Google speech on QTrobot. You can use any other service such as Whisper. Make sure that the transcript is provided for further processing correctly.

### Offline (Riva or Vosk)

Follow instrucitons for setting up [Riva or Vosk]](https://github.com/luxai-qtrobot/tutorials/tree/master/examples/offline_conversation). Again, make sure that the transcript from the ASR service of your choice is correctly passed for furhter processing.



## Vector store and embeddings

If you want to use RAG methods for language generation (optional) you need to setup an embeddings model and a vector store. Please activate the option `use_rag: 1` in the config file. If this is activated, please choose an option for `embeddingsengine` in the config file. In this example, we work with OpenAI embeddings (`embeddingsengine: "openai"`, online) or with *sentence-transformers/all-MiniLM-L6-v2* model  (`embeddingsengine: "hugging"`, offline). If RAG option is not chosen, the program will work as a simple ASR-NLG-TTS pipeline.

Other options:
   - `chunk_size: 250` specifes the size of chunks used for encoding
   - `chunk_overlap: 8` specifies whether and how much chi=unks should overlap.
   - `vectorstorename: "../your-vector-store-name"` specifies the folder name where a local copy of the vector store will be stored. The data will be encoded and stored in a vector store when RAG method is called for the first time. Later, a local copy can be used. Note: if data have changed, the vector store needs to be updated.
   - `knowledge_location: "path-to-your-data-folder"` specifies the path to the data files, the knowledge that you want to encode to be used with RAG.

In this example, we use `unstructured` library that is able handling multiple file types in one directory. We also use a simple chunking method that splits the data in chunks of 250 tokens. However, a smarter chunking method might be more useful in your application. Modify the `split_documents` method of the MemoryHandler class or ad another chunking method and use it in your pipeline.


## Languge generation

This example provides a simple interface for trying out online language generation with OpenAI models or offline with FastChat Vicuna-13b-v1.5 model. Please note that using offline language generation requires either a more powerfull hardware, such as QTrobot AI@Edge, or a smaler model (for example models from GPT4All). 

For using OpenAI models, set `chatengine: "chatgpt"` in the config file and provide your OpenAI key `key: ""`. Do not share your API key!

For using FastChat models, set `chatengine: "fastchat"` in the config file and leave the `key: ""` empty.


## Text-To-Speech

You can use many voices and languages for TTS. For languages available on your robot, you just need to specify the voice code in the config file. In our example, it is `tts_voice: 'en_US'`. You can modify pitch and speed of the TTS voince in the `__init__` method of `QTChatBot` class `status = self.speechConfig(self.tts_voice, 120, 80)`.


## Agent's personality

You can easily provide your social robot with different personalities and knowledge. In this example, we use a combination of RAG and prompt engineering methods to make conversations with QTrobot more personal and consistent. For example, if you ask the robot about its orgin, it should always tell th same story. 

The robot's identity can me modelled as a combination of self-awareness (knowledge about itself), knowledge about the world, knowledge about the other speaker (not handled here but we have done some prior work on [social proximity](https://github.com/svetaatluxai/socialproximity.git)), and memories about past conversations with the user (RAG techniques can be very useful for that! Just add a separate index for conversation logs and a procedure handling its usage).

### The robot's self-awareness
In this example, the origin of the robot is the planet Electoria. This information is stored in a file, and the agent's character can be easily changed by modifying the text in the file `data/knowlege/qtrobot-story.txt`. This message will be passed to the generative model as a `system_message`.

### The robot's knowledge about the world
In this example, we generated a small number of stories about the robot and stored them in the `data` folder. If you chose to work with RAG methods, the knowledge will be encoded and stored in a vector store. Each user's utterance will be then passed to the similarity search method. the retrieved context will be used as part of the next messages for answer generation. The good thing is that the knowledge can be stored in one language, and the answers can be generated in many languages supported by the LLM of your choice!

### The robot's memories

All conversations will be automatically stored in the `chatlog_path` folder set in [config file](config/conversation_with_memory.yaml) . You can use them for quality assessment and/or simulating long-term memory. 

