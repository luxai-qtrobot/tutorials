## Instalation overview
- **Create virtual environment**
- **install pytorch for jetson** 
- **install tensorflow for jetson**
- **install deepface**: `pip insatll deepface`

### steps: 
```bash
python -m pip install --upgrade pip
pip install -U testresources setuptools==65.5.0
pip install -r requirements.txt
```

### Sdd this to `LD_PRELOAD` to solve the issue of opencv import: 
```bash
 export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0:$LD_PRELOAD
```

