import csv

def write_to_csv(file_path, data, headers=None):
    try:
        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if headers:
                writer.writerow(headers)
            writer.writerows(data)
        print("Data successfully written to", file_path)
    except Exception as e:
        print("An error occurred:", e)

# Example data
data = [
    ["Name", "Age", "City"],
    ["Alice", 30, "New York"],
    ["Bob", 25, "Los Angeles"],
    ["Charlie", 35, "Chicago"]
]

# Example headers
headers = ["Name", "Age", "City"]

# File path to write the CSV file
file_path = "log_data/output.csv"

# Writing data to CSV file
write_to_csv(file_path, data, headers)