from fsm_backup import FSM
from data_hub_backup import DataHub
import time 

rate = 10

datahub = DataHub(rate)
fsm = FSM(datahub)

if __name__ == "__main__":
    while True:
        fsm.transition()

        time.sleep(0.1)