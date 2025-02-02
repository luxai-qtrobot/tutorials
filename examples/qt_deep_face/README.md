# Simple ROS wrapper for DeepFace library 
This is a simple ROS wrapper for [DeepFace](https://pypi.org/project/deepface/) library which publishes the regonized facial features 
on `/qt_deep_face/faces` topic. The published message is a JSON string which can be simply 
loaded using `json.loads(...)`.  

Here is an example of published JSON message for `emotion` action with `recognize_persons` is enabled: 

```json
[
    {
        "emotion": {
            "angry": 0.013950645890382422,
            "disgust": 2.2490248646898015e-08,
            "fear": 98.90159977655414,
            "happy": 0.19242573796725418,
            "sad": 0.37002987473846977,
            "surprise": 0.22720913022498723,
            "neutral": 0.29478262855468645
        },
        "dominant_emotion": "fear",
        "region": {
            "x": 438,
            "y": 346,
            "w": 57,
            "h": 57,
            "left_eye": null,
            "right_eye": null
        },
        "face_confidence": 0.97
    },
    {
        "name": "bob",
        "identity": "./db/bob_1.jpg",
        "distance": 0.42,
        "threshold": 0.6,
        "region": {
            "x": 438,
            "y": 346,
            "w": 57,
            "h": 57,
            }
        }    
    // ...
]
```

## published messages: 
- **`/qt_deep_face/faces`** : Detected facial festures in JSON string format 
- **`/qt_deep_face/image/out`**: Image output with landmarks for visualization 


## Installation
### Create virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate 
```

### Update pip in venv 
```bash
python -m pip install --upgrade pip
pip install -U testresources setuptools==65.5.0
```

### Install requrements:
```bash
pip install -r requirements-RDV2AI@Edge.txt
```

### Add the following line to `~/.bash_aliases` to solve the issue of opencv import: 
```bash
 export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0:$LD_PRELOAD
```


## Usage
By defualt the `emotion` recogntion is enable in `QTDeepFace` insatnce from `qt_deep_face.py`: 

```python
dp = QTDeepFace(actions=['emotion'])
#...
```
running the code as it is, will published recognized emotions of multiple faces. 

You can also use ROS `image_view` to see the image output with landmarks: 
```bash
rosrun image_view image_view image:=/qt_deep_face/image/out
```

**Note:** IF `image_view` is not installed on QTPC, you can simple install it using `sudo apt install ros-noetic-image-view`. 


If you would like to use the code for person recognition, please follow these steps: 

### Put the images (profile photos) of the persons in a folder
- create a folder e.g. `/home/qtrobot/persons`
- copy the `jpg` photos of the persosn in the folder: e.g. `bob_1.jpg`, `sally_1.jpg`, etc...

**Note:** keep the name of the images in this format: `<name>_1.jpg`, `<name>_2.jpg`. You can add multiple photos of the same person. 

### Enable `recognize_persons` in `QTDeepFace` insatnce from `qt_deep_face.py`

enable the `recognize_persons=True` and set the `persons_db_path` to your persons database folder
```python
dp = QTDeepFace(actions=['emotion'], recognize_persons=True, persons_db_path="/home/qtrobot/persons")
``` 

## Other paramters and optimization
You can set the following paramters in `QTDeepFace` class to optimize the performance. For example, if you need very high precision of face detector, for example to detect faces from far distance, you can set `detector_backend='retinaface'`. However, this also loads more the GPU. 
using `'ssd'` fort `detector_backend` provide good balance between accuracy and performance.  

Please check [DeepFace](https://pypi.org/project/deepface/) for comparison between, detectors, models! 

```python
    actions=['emotion']       # actions : ['age', 'gender', 'race', 'emotion'])
    recognize_persons=False   # enable person recognition
    persons_db_path="./"      # database folder for person recognition 
    frame_rate=5              # processing framerate (use to not overload system)
    model_name='VGG-Face',    # Face recognition models: "VGG-Face", "Facenet", "Facenet512", "OpenFace", "DeepFace", "DeepID", "ArcFace", "Dlib", "SFace", "GhostFaceNet"
    detector_backend='ssd'   # Face Detectors: 'opencv', 'ssd', 'dlib', 'mtcnn', 'fastmtcnn', 'retinaface', 'mediapipe', 'yolov8', 'yunet', 'centerface',
```
  
