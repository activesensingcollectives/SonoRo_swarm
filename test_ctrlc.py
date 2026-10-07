import subprocess
import time
import concurrent.futures

def run():
    print("starting ssh")
    proc = subprocess.Popen(["sleep", "10"])
    proc.wait()
    print("ssh done")

try:
    with concurrent.futures.ThreadPoolExecutor() as ex:
        f = ex.submit(run)
        f.result()
except KeyboardInterrupt:
    print("\ncaught ctrl+c in main")
    print("running stop command")
    subprocess.run(["echo", "stopped"])
