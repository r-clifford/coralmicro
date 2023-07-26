import csv
from pathlib import Path



def get_sample_rate(filename):
    data = []    
    try:
        with open(filename, newline='') as f:
                reader = csv.reader(f, delimiter=",")
                for row in reader:
                    data.append([float(x) for x in row])
    except:
        print(f"Failed to read file: {filename}")
        return
    try:
        first = data[0]
        last = data[-1]
        start = first[0]
        end = last[0]
        deltaT = (end - start) * 1e-3
        totalSamples = len(data)
        sampleRate = totalSamples / (deltaT)
    except IndexError:
        sampleRate = 0.0
    print(f"File: {filename} = {sampleRate}Hz")

if __name__ == "__main__":
    cwd = Path(".")
    paths = [p for p in cwd.iterdir() if p.is_file()]
    for path in paths:
        get_sample_rate(path)
    
