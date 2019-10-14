#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt


def main(filename):
    # Read the CSV file to a DataFrame, and set the proper column headers
    df = pd.read_csv(filename, index_col=0, skiprows=6)
    df.rename(columns=lambda x: x.split(' ')[0], inplace=True)

    # Create an iterator which yields one row per call
    # Use with e.g. in a loop
    # rows = df.iterrows()
    # for row in rows:

    # Plot two sample points against one another
    fig, ax = plt.subplots()
    df.plot(y='47', ax=ax)
    df.plot(y='r<ef', ax=ax)
    plt.show()


if __name__ == "__main__":
    filename = 'miniMea.csv'
    main(filename)
