import csv

class CsvWriter:
    def __init__(self, filename):
        self.filename = filename
        self.csvList = []

    def append_for_csv(self, append_list):
        self.csvList.append(append_list)

    def write_csv(self):
        print(' -   -   -   -   -   -   -   -   -   -   -   -  ')
        print(self.csvList)
        with open(self.filename, 'w', newline='') as new_file:
            writer = csv.writer(new_file, delimiter=',')
            for row in self.csvList:
                writer.writerow(row)

