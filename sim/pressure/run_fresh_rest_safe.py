"""Filesystem-only entry point for frozen fresh-rest producer, including new output dirs."""
import pathlib,sys
import run_fresh_rest_controller as frozen

def run(output,scenario='nominal',dt=.001,duration=3.):
 output=pathlib.Path(output)
 output.parent.mkdir(parents=True,exist_ok=True)
 return frozen.run(output,scenario,dt,duration)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else 3.)
