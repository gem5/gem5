import pandas as pd

print("Loading instruction trace...")

# Check how many columns exist in the CSV to determine if we have assembly text yet
with open('m5out/ctx_inst_trace.csv', 'r') as f:
    first_line = f.readline()
    num_cols = len(first_line.split(','))

# Load data based on the format
if num_cols == 4:
    df = pd.read_csv('m5out/ctx_inst_trace.csv', names=['Tick', 'CoreID', 'PC', 'InstName'])
    instruction_col = 'InstName'
    print("Detected Assembly Mnemonics (add, ldr, etc.)!")
else:
    df = pd.read_csv('m5out/ctx_inst_trace.csv', names=['Tick', 'CoreID', 'PC'])
    # Format the raw PC address to hex
    df['PC_Hex'] = df['PC'].apply(lambda x: f"Inst_0x{int(x, 16) if isinstance(x, str) else int(x):X}")
    instruction_col = 'PC_Hex'
    print("Warning: Currently grouping by raw PC address. Update C++ to see 'add', 'ldr' names.")

if df.empty:
    print("CSV is empty!")
    exit()

# 1. Identify distinct Context Switches
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['Switch_ID'] = (df['Tick_Diff'] > 10_000_000).cumsum() + 1

# 2. Create the Table (Rows: Switch_ID, Columns: Instruction, Values: Count)
# pd.crosstab automatically counts the occurrences!
table = pd.crosstab(df['Switch_ID'], df[instruction_col])

# 3. Add the Total Row (-1) across all context switches
table.loc[-1] = table.sum(axis=0)

# 4. Sort index so the -1 row stays exactly at the bottom
table = table.sort_index(key=lambda idx: idx.map(lambda x: 9999999 if x == -1 else x))

# 5. Save the table
output_file = "instruction_usage_table.csv"
table.to_csv(output_file)

print(f"\nTable successfully saved to {output_file}")
print("\nPreview of the table:")
print(table.iloc[:, :5].head(5)) # Print a small preview of the first 5 columns
