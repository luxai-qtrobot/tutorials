---
id: qt_ai_assistant_online
title: "QTrobot: Your Online AI Data Assistant"
hide_table_of_contents: true
pagination_label: QTrobot-Your Online AI Data Assistant
---


import Icon from '@mui/material/Icon';
import GitHubIcon from '@mui/icons-material/GitHub';

:::info Overview
<br/> <Icon> track_changes </Icon> &nbsp;**Goal:**&nbsp; *This project demonstrates how to use online LLM and RAG to build conversational data assistant*
<br/> <Icon> code </Icon> &nbsp;**Source code:**&nbsp; *[Check the source code here](https://github.com/luxai-qtrobot/tutorials/tree/master/demos/qt_ai_data_assistant_online)*
:::

**The Online AI Data Assistant** is a flexible, cloud-enhanced version of the QT AI Assistant, designed for QTrobot models that do not include dedicated GPU hardware, such as QTrobot-i5 and QTrobot-i7. Built upon the powerful LlamaIndex framework, this demo enables QTrobot to serve as a conversational AI assistant that can ingest, index, and intelligently respond to user queries over custom documents and online content.



## Table of Contents

- [QTrobot: Your AI Data Assistant](#qtrobot-your-ai-data-assistant)
  - [Features](#features)
  - [Technologies Used](#technologies-used)
  - [Getting Started](#getting-started)
    - [Installation](#installation)
    - [Usage](#usage)
      - [Run the QTrobot Data Assistant with demo data:](#run-the-qtrobot-data-assistant-with-demo-data)
      - [Run the QTrobot Data Assistant with your own data:](#run-the-qtrobot-data-assistant-with-your-own-data)
      - [Pause and Resume the conversation with voice command](#pause-and-resume-the-conversation-with-voice-command)
    - [Customization](#customization)
      - [Using a different converation language:](#using-a-different-converation-language)
      - [Using different document formats:](#using-different-document-formats)
      - [Customizing the QTrobot's Role](#customizing-the-qtrobots-role)
        - [Why Customizing the Role is Interesting](#why-customizing-the-role-is-interesting)
      - [Choosing a different LLM model for conversation:](#choosing-a-different-llm-model-for-conversation)      
      - [Enabling scene undertanding](#enabling-scene-undertanding)
      - [Disabling Retrieval-Augmented Generation](#disabling-retrieval-augmented-generation)
      - [Storing and restoring conversations](#storing-and-restoring-conversations)
      - [Command-line parameters](#command-line-parameters)
  - [License](#license)

## Features

- **Retrieval-Augmented Generation (RAG):** Enhances QTrobot's ability to provide accurate and context-aware answers by retrieving relevant information from user-provided documents (e.g. `.txt`, `.pdf`, `.docx`, `.md`) and simple web pages. This retrieval is powered by semantic search over documents indexed in the cloud using AstraDB.

- **Multilingual Query Support:** Supports questions in more than 14 languages, regardless of the language used in the original data source. Users can freely interact with their content in the language they are most comfortable with.

- **Online Large Language Models (LLMs):** Integrates with a variety of cloud-based LLMs (e.g. OpenAI, Groq) to generate fluent, context-aware responses. The modular design allows easy expansion to new LLM providers via LlamaIndex.

- **Online Automatic Speech Recognition (ASR):** Offers real-time speech-to-text using multiple online services—including **Groq (Whisper)**, **Azure Speech Service**, and **Google Speech Service**. Users can select the ASR provider that best matches their needs for performance, cost, or regional support.

- **Web Data Ingestion:** In addition to local documents, this version can **scrape and index web pages**, allowing dynamic expansion of its knowledge base without manual upload steps.

- **Flexible, Cloud-Based Architecture:** This assistant is optimized for **resource-constrained robots** and can run on **QTrobot-i5 and QTrobot-i7** without dedicated GPU hardware. Its modular design also makes it ideal for evolving deployments and diverse connectivity scenarios.

- **Provider Choice & Cost Optimization:** Gives users control over which ASR, LLM, or vector store service they use—enabling smarter, **cost-effective customization** of their AI assistant setup.



## Technologies Used

Following is an overview of the key technologies and services employed to implement each feature of the **QTrobot: Your Online AI Data Assistant**. These tools were selected for their flexibility, extensibility, and ability to operate efficiently in resource-constrained, GPU-less environments while offering robust cloud integration.

- **Online Large Language Models (LLMs):** Integration with cloud-based models such as **GPT-4**, **GPT-4o**, and **GPT-4o-mini** from OpenAI, as well as **LLaMA 3**, **LLaMA 3.1**, **LLaMA 4**, and **Google Gemma2** via **Groq API**. These models provide high-quality, multilingual, and context-aware natural language responses. Integration is handled through [**LlamaIndex**](https://www.llamaindex.ai/), ensuring abstraction and extensibility across LLM providers.

- **Online Automatic Speech Recognition (ASR):** Supports real-time transcription using multiple online ASR services including **Groq Whisper**, **Azure Speech Service**, and **Google Speech Service**. These services collectively support over 100 languages and dialects, enabling versatile, multilingual voice interaction. The user can configure the assistant to use their preferred provider, balancing cost, performance, and accuracy needs.

- **Online Automatic Speech Recognition (ASR):** Supports real-time transcription using multiple online ASR services including **Groq Whisper**, **Azure Speech Service**, and **Google Speech Service**. These services collectively support over 100 languages and dialects, enabling versatile, multilingual voice interaction. 

- **Offline Voice Activity Detection (VAD):** Utilizes [**Silero VAD**](https://github.com/snakers4/silero-vad), a highly accurate and lightweight offline voice activity detector, to identify when a user is actively speaking. This allows the system to initiate ASR streaming **only during detected speech**, significantly reducing unnecessary API usage, network traffic, and associated costs.

- **Vector Store (RAG Backend):** [**AstraDB**](https://www.datastax.com/astra) is used for storing document embeddings and enabling fast semantic search. AstraDB is a highly scalable, serverless vector database built on Apache Cassandra. Thanks to [**LlamaIndex's** vector store abstraction](https://docs.llamaindex.ai/en/stable/module_guides/storing/vector_stores/), the implementation can be easily adapted to alternative vector databases such as Pinecone, Qdrant, Chroma, or FAISS.

- **Data Ingestion & Web Integration:** Document ingestion and chunking are powered by [**LlamaIndex**](https://www.llamaindex.ai/), supporting formats such as `.pdf`, `.txt`, `.docx`, and `.md`. Additionally, it includes a web connector layer that allows scraping and indexing of simple HTML pages. This can be extended via tools like **SpiderWeb** and other [LlamaIndex web connectors](https://docs.llamaindex.ai/en/stable/examples/data_connectors/WebPageDemo/) to handle richer web content extraction.

- **Text-to-Speech (TTS):** The assistant uses the default **Acapela TTS engine** provided by QTrobot for speech synthesis. This engine supports multiple voices and languages and ensures real-time verbal responses fully offline.

- **Runtime & Deployment:** All services and processing run natively on **QTrobot-i5** and **QTrobot-i7** platforms without requiring GPU support. The architecture is fully self-contained and optimized for embedded execution, ensuring that the robot remains responsive and operational without dependency on external compute infrastructure.



## Getting Started
The QTrobot Data Assistant can run on all variants of QTrobot for Research version.

### Installation
:::info Notice
 - The project's source code is available in the *[LuxAI Tutorials Github Repository](https://github.com/luxai-qtrobot/tutorials/tree/master/demos/qt_ai_data_assistant_online)*. To get the latest version of the code on your **QTPC**: `cd ~/robot/code/tutorials/; git pull`
 - Ensure that *software*  repository on **QTPC** is updated and service files for `qt_respeaker_app` is installed in your catkin workspace: 
    - `cd ~/robot/code/software/; git pull`
    - `cd ~/catkin_ws/; ln -s ~/robot/code/software/headers/qt_respeaker_app/ ./src`
    - `cd ~/catkin_ws/; catkin_make`
- Throughout this documentation, we will refer to *"~/robot/code/tutorials/demos/qt_ai_data_assistant_online"* as *"qt_ai_data_assistant_online"*.
:::

The installation process involves three main steps: ***(1) Installing Nvidia Riva ASR***, ***(2) Installing Ollama and required models*** and ***(3) Installing python packages*** 



1. **Install Nvidia Riva ASR:**
   - Sign in to your [Nvidia NGC](https://ngc.nvidia.com/signin) portal (sign up if you don't have an account).
   - Follow the instructions for [Generating Your NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key) to obtain an API key.
   - Use your API key to log in to the NGC registry by entering the following command and following the prompts.

     **Note:** The `ngc` CLI is already installed on the QTrobot, so no need to install it.
     ```bash
     ngc config set
     ```
   - Download the Embedded Riva start scripts to the `~/robot` folder using the following command:
     ```bash
     cd ~/robot/      
     ngc registry resource download-version nvidia/riva/riva_quickstart_arm64:2.14.0
     ```
   - Modify `~/robot/riva_quickstart_arm64_v2.14.0/config.sh` to disable unnecessary services, keeping only `service_enabled_asr` enabled. The relevant part of `config.sh` should look like this:
     ```bash
     # ...
     # Enable or Disable Riva Services
     service_enabled_asr=true
     service_enabled_nlp=false
     service_enabled_tts=false
     service_enabled_nmt=false
     # ...
     ```
   - Update the `~/robot/riva_quickstart_arm64_v2.14.0/riva_init.sh` script to resolve the Nvidia NGC CLI version issue by changing the version from `3.26.0` to `3.63.0`. Replace the URL on line 170 with:
     ```bash
      # Original line:
      # https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/3.26.0/files/ngccli_arm64.zip

      # Updated line:
      https://api.ngc.nvidia.com/v2/resources/nvidia/ngc-apps/ngc_cli/versions/3.63.0/files/ngccli_arm64.zip
     ``` 
   - Initialize the Riva ASR container. This may take around 30 minutes, depending on your internet speed.
     ```bash
     cd ~/robot/riva_quickstart_arm64_v2.14.0
     bash ./riva_init.sh
     ```

2. **Install ollama and pull the required models:**
    - We have prepared a script (`install_ollama.sh`) to download and install the appropriate version of ollama on QTrobot's Jetson AGX Orin. To install Ollama, execute the following command:
      ```bash
      cd qt_ai_data_assistant_online
      sudo bash ./scripts/install_ollama.sh
      ```

    - Next, download the required models. These models are approximately 6 GB in size, so the download may take some time. If the download process is interrupted, simply rerun the command to resume.

      ```bash
      ollama pull llama3.1:latest
      ollama pull mxbai-embed-large:latest
      ```
    
    - Verify that all the required models have been downloaded correctly by using the `ollama list` command. The output should look similar to the following:
      ```bash
      NAME                    	ID          	SIZE  	MODIFIED      
      mxbai-embed-large:latest	468836162de7	669 MB	3 minutes ago
      llama3.1:latest         	f66fc8dc39ea	4.7 GB	10 minutes ago
      ```
    - Test the llama3.1 model from the command line using the following command. Please note that it may take some time for Ollama to load the model for the first time. If Ollama fails with a timeout message during the initial load, simply rerun the command. Subsequent model loads will be much faster.:
      ```tex
      $ ollama run llama3.1
      >>> who are you?
      I'm an artificial intelligence model known as Llama. Llama stands for "Large Language Model Meta AI."
      >>> Send a message (/? for help)
      ```
      To exit the chat, press `ctrl+c` or type `/bye`.

3. **Install required python packages:**
    - Create a python3 virtual environment and install pip packages:    
      ```bash
      cd qt_ai_data_assistant_online
      python3 -m venv venv
      source venv/bin/activate 
      pip install --upgrade pip
      pip install -r requirements.txt
      ```


### Usage

***Note:***: *An internet connection is required the first time you run the QTrobot Data Assistant to download the necessary models, including NLTK, RetinaFace, and VGG-Face.*


First, ensure that the Riva ASR Docker service is running. If Riva is not active, it may take 1 to 2 minutes to initialize.
```bash
cd ~/robot/riva_quickstart_arm64_v2.14.0
bash ./riva_start.sh ./config.sh -s
```

#### Run the QTrobot Data Assistant with demo data:
The repository includes a demo PDF file detailing the QTrobot research variants and specifications. To run the project with this demo document: 
```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py
```
Wait a few seconds for the script to initialize and load the document. Once it's ready, you can begin conversing with the QTrobot. For example, you might ask, *"What are all your variants?"* or *"How can I program QTrobot?"* You can also ask questions unrelated to the document, such as, *"What are the official languages of Luxembourg?"*

#### Run the QTrobot Data Assistant with your own data:
To run the QTrobot Data Assistant with your own data, first create a folder to store your documents:
```bash
mkdir ~/Documents/qt-assistant-data
```
Copy your PDF file(s) into your project's document folder (`~/Documents/qt-assistant-data`). For example, you can use one of your research papers. Then, simply start the `qt_ai_data_assistant_online.py` script and provide the path to your document folder.

```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py -d ~/Documents/qt-assistant-data
```
Wait a few seconds for the script to initialize and load the document. Once it's ready, you can ask questions about the content of your PDF.    

#### Pause and Resume the conversation with voice command
You can ask QTrobot to pause the conversation and remain unresponsive until explicitly instructed to resume. This can be done through natural language commands:

- To pause the conversation, simply say: *"Hold on for a minute"* or *"Stay silent."*
- To resume the conversation, explicitly say: *"Restart the conversation"* or *"Let's talk."*

### Customization
You can customize the demo project in various ways to suit your needs:

#### Using a different converation language:
QTrobot Data Assistant allows users to query their data in a language different from the one used in the provided document. To do this, you must enable your desired language in your Riva ASR installation and ensure that the corresponding language is also configured for QTrobot's Text-to-Speech (TTS) system. 

Riva ASR supports a variety of languages including English, German, French, Italian and many more. Check the [Riva ASR supported languges](https://docs.nvidia.com/deeplearning/riva/user-guide/docs/asr/asr-overview.html#pretrained-asr-models) for complete list. 

To enable a specific language in Riva ASR, you need to modify the `~/robot/riva_quickstart_arm64_v2.14.0/config.sh` file to add your language code and then re-initialize Riva ASR. For example, to enable French (fr-FR), modify the `asr_language_code` parameter to include `fr-FR`:

```bash
# For multiple languages enter space separated language codes.
asr_language_code=("en-US" "fr-FR")
```

Next, stop the currently running instance of Riva and re-initialize the service:

```bash
cd ~/robot/riva_quickstart_arm64_v2.14.0
bash ./riva_stop.sh
bash ./riva_init.sh
```
Then restart the Riva ASR:
```bash
cd ~/robot/riva_quickstart_arm64_v2.14.0
bash ./riva_start.sh ./config.sh -s
```

Once Riva ASR is running, you can start the `qt_ai_data_assistant_online.py` script and provide the conversation language code:
 
```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py --lang fr-FR
```

The `--lang <lang code>` parameter sets both the ASR and TTS languages to the specified language.

***Note:*** *If the TTS language is not installed on your QTrobot, please contact [support@luxai.com](malito:support@luxai.com) to obtain the necessary language files.* 



#### Using different document formats: 
By default QTrobot Data Assistant loads PDF (`.pdf`) documents from the specified directory. It uses the [LlamaIndex Data Framework](https://www.llamaindex.ai/) to support various document formats such as `.txt`, `.docx` and `.md`. You can specify the document formats and the maximum number of files to load using the appropriate parameters.

For example, to load a maximum of 5 PDF and DOCX files from `~/Documents/qt-assistant-data`, use the following commands:
```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py --formats .pdf .docx --max-docs 5
``` 

#### Customizing the QTrobot's Role
You can customize the role and behavior of QTrobot by modifying the system role in the LLM prompts. This allows you to adapt QTrobot to different scenarios, making it a versatile assistant tailored to specific tasks or environments. By changing the system role, you can guide how QTrobot responds, behaves, and interacts with users.

##### Why Customizing the Role is Interesting
Customizing the system role allows you to adapt QTrobot to a wide range of applications beyond its default educational and research-focused roles. Whether you need QTrobot to act as a receptionist, a healthcare assistant, a tourist guide, or any other role, this flexibility makes QTrobot a powerful tool in various environments. By fine-tuning the system role, you can ensure that QTrobot meets the specific needs of your users, providing a more personalized and effective experience. Additionally, by leveraging Retrieval-Augmented Generation (RAG), QTrobot can provide highly relevant and contextual responses based on user-provided documents and data, enhancing its utility in specialized roles.

The default system role is defined in the `llm_prompts.py` file: 
```python
ConversationPrompt = {
    'system_role': '''     
     You are a humanoid social robot assistant named "QTrobot". You can hear and talk. You are designed to support various use cases, including the education of children with autism and other special needs, as well as human-robot interaction research and teaching.

    Follow these guidelines when answering questions:
    - Always respond in one or two brief sentences. Keep your sentences as short as possible.
    ...    
    '''
}
```
To modify QTrobot’s role, you can edit the relevant part of `system_role` section in the `llm_prompts.py` file. By changing the system role, you can assign QTrobot a new identity and set of behaviors. Here are some examples of how you might customize QTrobot's role, emphasizing its ability to use Retrieval-Augmented Generation (RAG) to provide insightful responses based on user-provided data: 

- **Example 1: Receptionist**
 
  Transform QTrobot into a virtual receptionist who can greet visitors, provide information, and answer queries based on company documents or FAQs:
  ```python
  ConversationPrompt = {
      'system_role': '''     
      You are a receptionist named "QTrobot". Your job is to greet visitors, answer their questions, and provide information based on the company’s documents.

      Follow these guidelines when answering questions:
      - Always respond in one or two brief sentences. Keep your sentences as short as possible.
      ...    
      '''
  }
  ```

- **Example 2: Tourist Guide**
 
  QTrobot can act as a virtual tourist guide, offering information about local attractions, history, and travel tips by pulling data from guidebooks, maps, and other resources
  ```python
  ConversationPrompt = {
      'system_role': '''     
      You are a tourist guide named "QTrobot". Your role is to provide tourists with information about local attractions, history, and travel tips based on guidebooks and other documents.

      Follow these guidelines when answering questions:
      - Always respond in one or two brief sentences. Keep your sentences as short as possible.
      ...    
      '''
  }
  ```

- **Example 3: Research Conference Center Assistant**
  
  Transform QTrobot into a virtual conference assistant that helps attendees navigate the event, find sessions, and get information about workshops and presentations:
  ```python
  ConversationPrompt = {
      'system_role': '''     
      You are a conference assistant named "QTrobot" at a research conference center. Your role is to assist attendees by providing information about the venue, schedule, workshops, and research presentation sessions.       
      Provide clear and concise information about the conference schedule, including session times and locations only from the provided documents.
      Use a polite and professional tone.

      Follow these guidelines when answering questions:
      - Always respond in one or two brief sentences. Keep your sentences as short as possible.
      ...    
      '''
  }
  ```


#### Choosing a different LLM model for conversation:
By default, QTrobot Data Assistant uses the  [**Mata Llama 3.1**](https://llama.meta.com/) llm for conversation.  However, you can experiment with different LLMs, such as  [Google Gemma2](https://blog.google/technology/developers/google-gemma-2/) to explore varying conversational styles, performance, or domain-specific knowledge. You can experiment with these models via the Ollama interface. To use a different model, first pull the model using the Ollama command: 

```bash
ollama pull gemma2  
```
Next, start the `qt_ai_data_assistant_online.py` script and specify the LLM model as a parameter:

```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py --llm gemma2
```

#### Enabling scene undertanding:
QTrobot uses the [moondream](https://moondream.ai) Visual Language Model (VLM) to analyze scenes, recognize objects, and infer context, allowing it to provide responses based on visual cues. By default, this feature is not enabled. Follow these steps to enable scene understanding and converse with QTrobot about what it sees through its camera feed.

First, pull the Moondream model using the Ollama command. The model is approximately 1.7GB, so downloading may take some time:
```bash
ollama pull moondream 
```

Next, start the `qt_ai_data_assistant_online.py` script and enable scene understanding with the following parameter:

```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py --enable-scene
```

When enabled, the `SceneDetection` module will periodically query the VLM (by default every 10 seconds) and use the scene description as additional context for the LLM. After enabling the scene feature in `qt_ai_data_assistant_online.py`, wait for a short time and then start interacting with QTrobot. You can ask questions like, *"Tell me what you see,"* or *"How many people do you see now?"*

***Note:*** *Enabling scene understanding may introduce a slight delay in QTrobot's response time due to increased GPU usage and inference time required by Ollama during LLM and VLM processing at the same time*


#### Disabling Retrieval-Augmented Generation:
In some cases, you may not need to use the Retrieval-Augmented Generation (RAG) feature, especially if the application does not require any user-provided documents or knowledge. Disabling RAG allows QTrobot to interact directly with the LLM for faster, more fluent conversations, as it reduces the processing overhead and latency associated with retrieving and integrating external data.

To disable RAG, simply start the qt_ai_data_assistant_online.py script with the `--disable-rag` parameter:
```bash
cd qt_ai_data_assistant_online
source venv/bin/activate
python src/qt_ai_data_assistant_online.py --disable-rag
```
When RAG is disabled, QTrobot will no longer retrieve information from documents or external sources, and all responses will be generated directly by the LLM based on the content provided in the `system_role`. This can improve response times and is ideal when external knowledge retrieval is not needed for your use case.

#### Storing and restoring conversations
To save the entire conversation and restore it later, simply run the `qt_ai_data_assistant_online.py` script and specify the path to the JSON file where the conversation memory will be stored.
- If the JSON file already exists, the conversation memory will be loaded from it at the start.
- If the file does not exist, a new one will be created to store the conversation.

Example usage:
```bash 
python qt_ai_data_assistant_online.py --mem-store ~/chat_store.json
```

#### Command-line parameters
```bash
python qt_ai_data_assistant_online.py --help

usage: qt_ai_data_assistant_online.py [-h] [-d DOCS] [--formats FORMATS [FORMATS ...]] [--max-docs MAX_DOCS] [--lang LANG] [--llm LLM] [--mem-store MEM_STORE] [--enable-scene] [--disable-rag]

optional arguments:
  -h, --help            show this help message and exit
  -d DOCS, --docs DOCS  Path to the folder containg documents to load
  --formats FORMATS [FORMATS ...]
                        document formats ('.txt' '.pdf', '.docx', '.md').
  --max-docs MAX_DOCS   maximum number of docuemnt files load
  --lang LANG           Conversation language (ar-AR, en-US, en-GB, de-DE, es-ES, es-US, fr-FR, hi-IN, it-IT, ja-JP, ru-RU, ko-KR, pt-BR, zh-CN)
  --llm LLM             LLM model to use. default is llama3.1.
  --mem-store MEM_STORE
                        path to a json file (e.g. ./chat_store.json) to store the and restore the conversation memeory
  --enable-scene        Enables camera feed scene processing
  --disable-rag         Disable Retrieval-Augmented Generation

example: qt_ai_data_assitant.py -d ~/my-docuemnts --formats .pdf .docx --max-docs 3 --lang en-US
```

## License
This project is licensed under the MIT License. See the [LICENSE](https://github.com/luxai-qtrobot/tutorials/blob/master/demos/qt_ai_data_assistant_online/LICENSE) file for details.

### Copyright
© 2024 LuxAI S.A. All rights reserved.
