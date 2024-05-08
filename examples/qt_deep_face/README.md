# Simple ROS wrapper for DeepFace library 
This is a simple ROS wrapper for [DeepFace](https://pypi.org/project/deepface/) library which publishes the regonized facial features 
on `/qt_deep_face/faces` topic. The published message is a JSON string which can be simply 
loaded using `json.loads(...)`.  

Here is an example of published JSON message for `emotion` action: 

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
    // ...
]
```

## published messages: 
- **`/qt_deep_face/faces`** : Detected facial festures in JSON string format 
- **`/qt_deep_face/image:o`**: Image output with landmarks for visualization 


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
