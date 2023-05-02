#!/usr/bin/env python3

def is_run_in_DS_arm64_darwin_architecture() -> bool:
    """
    Check if code is executed inside a Dockerized-SNOW container for arm64-Darwin architecture
    """
    from os import getenv
    return getenv('DS_IMAGE_ARCHITECTURE') == 'arm64-Darwin'