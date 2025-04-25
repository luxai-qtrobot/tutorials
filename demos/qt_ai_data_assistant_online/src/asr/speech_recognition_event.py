# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from enum import Enum


class SpeechRecogntionEvent(Enum):
    STARTED = 1
    RECOGNIZING = 2
    RECOGNIZED = 3
    STOPPED = 4
    CANCELED = 5
