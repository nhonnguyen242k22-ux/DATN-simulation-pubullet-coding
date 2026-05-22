import sys

def read_thesis():
    with open('bionic_hand_thesis.txt', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
        
    for i, line in enumerate(lines):
        if 'D-H table' in line or 'Denavit' in line or 'D-H parameter' in line:
            print(f"--- MATCH AT LINE {i} ---")
            start = max(0, i - 5)
            end = min(len(lines), i + 25)
            for j in range(start, end):
                print(f"{j}: {lines[j].strip()}")
            print("-" * 40)

if __name__ == '__main__':
    read_thesis()
