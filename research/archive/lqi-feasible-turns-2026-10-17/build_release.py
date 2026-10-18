"""Build a non-instrumented, 25 deg/s bounded browser candidate separately."""
import build
import os
if __name__=='__main__':
    build.HERE=build.HERE/os.environ.get('RELEASE_DIR','deployment')
    build.HERE.mkdir(exist_ok=True)
    build.main(instrument=False,deployment=True,handover=os.environ.get('HANDOVER_RAMP')=='1',deployment_limit=int(os.environ.get('RELEASE_RATE','25')))
