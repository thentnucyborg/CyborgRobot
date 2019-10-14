import time

import numpy as np
import pandas as pd

import system.settings as settings


class FileServer:
    def __init__(self, loop_function, presenter):
        self.file_path = settings.NEURAL_DATA_FILE
        self.skip_rows = 0
        self.loop_function = loop_function
        self.presenter = presenter

    def spike_detection(self, data):
        n_rows, n_columns = data.shape
        spikes = np.zeros((n_rows, n_columns))
        volt = np.zeros((n_rows, n_columns))
        for i in range(n_rows):
            for j in range(n_columns):
                volt[i][j] = data.iloc[i][j]
                if volt[i][j] <= settings.THRESHOLD:
                    spikes[i][j] = 1
        return spikes, volt

    def read_CSV(self, N_ROWS):
        lines = pd.read_csv(self.file_path, skiprows=self.skip_rows, nrows=N_ROWS, index_col=0)
        self.skip_rows += N_ROWS
        return self.spike_detection(lines)

    def loop(self):
        frame_time = 1.0 / settings.LED_REFRESHES_PER_SECOND
        n_rows = int(10000 * frame_time)
        print(n_rows)
        while self.presenter.running():
            past_time = time.time()
            if settings.NEURAL_DATA_TYPE == 'frequency':
                '''
                The code below will read n_rows to do spike detection and will give out
                the frequency of spikes in a time interval. This runs way too slow so we wont use it

                spike, _ = self.read_CSV(n_rows)
                processed_data = [0] * settings.NEURAL_ELECTRODES_TOTAL
                for i in range(len(spike)):
                    for j in range(len(spike[i])):
                        if spike[i][j]:
                            processed_data[j] += 1
                '''
                n_rows = 1
                _, processed_data = self.read_CSV(n_rows)
                processed_data = processed_data[0] * -1
            elif settings.NEURAL_DATA_TYPE == 'intensity':
                n_rows = 1
                _, processed_data = self.read_CSV(n_rows)
                processed_data = processed_data[0] * -1
            self.loop_function(processed_data)
            delta = time.time() - past_time
            sleep_time = frame_time - delta
            if sleep_time < 0:
                print("Can't keep up!")
                sleep_time = 0
            time.sleep(sleep_time)


if __name__ == '__main__':
    pass
