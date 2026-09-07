import pandas as pd
import numpy as np

# تنظیم مسیر فایل‌ها در پوشه m5out
INTERVALS_FILE = 'm5out/ctx_switch_intervals.csv'
TRACE_FILE = 'm5out/ctx_inst_trace.csv'
OUTPUT_FILE = 'm5out/instruction_usage_table_exact.csv'

print("Loading intervals and instruction trace from m5out/...")

# ۱. بارگذاری بازه‌های دقیق کانتکست سوییچ‌ها (۱۴۹۹ سطر)
intervals = pd.read_csv(INTERVALS_FILE, names=['StartTick', 'EndTick']).dropna()
intervals['Switch_ID'] = np.arange(1, len(intervals) + 1)
total_switches = len(intervals)
print(f"Loaded {total_switches} context switch intervals.")

# ۲. بارگذاری دستورات ثبت‌شده
with open(TRACE_FILE, 'r') as f:
    first_line = f.readline()
    num_cols = len(first_line.split(','))

if num_cols == 4:
    df_inst = pd.read_csv(TRACE_FILE, names=['Tick', 'CoreID', 'PC', 'InstName'])
    inst_col = 'InstName'
else:
    df_inst = pd.read_csv(TRACE_FILE, names=['Tick', 'CoreID', 'PC'])
    df_inst['PC_Hex'] = df_inst['PC'].apply(lambda x: f"Inst_0x{int(x, 16) if isinstance(x, str) else int(x):X}")
    inst_col = 'PC_Hex'

if df_inst.empty:
    print("Error: m5out/ctx_inst_trace.csv is empty!")
    exit()

# ۳. تطبیق دقیق Tick هر دستور با بازه‌های StartTick و EndTick
ticks = df_inst['Tick'].values
assigned_switch = np.full(len(df_inst), -1, dtype=int)

print("Mapping instructions to exact interval windows...")
for _, row in intervals.iterrows():
    s_id = int(row['Switch_ID'])
    start = row['StartTick']
    end = row['EndTick']
    
    # پیدا کردن دستوراتی که درون پنجره زمانی این سوییچ اجرا شده‌اند
    mask = (ticks >= start) & (ticks <= end)
    assigned_switch[mask] = s_id

df_inst['Switch_ID'] = assigned_switch

# حذف دستوراتی که خارج از بازه‌ها بودند (در صورت وجود)
df_inst = df_inst[df_inst['Switch_ID'] != -1]

# ۴. ساخت جدول فراوانی دستورات (Cross-tabulation)
table = pd.crosstab(df_inst['Switch_ID'], df_inst[inst_col])

# اطمینان از وجود تمام ۱۴۹۹ سطر (حتی سوییچ‌هایی با اجرای ۰ دستور)
table = table.reindex(index=np.arange(1, total_switches + 1), fill_value=0)

# ۵. اضافه کردن سطر جمع کل (-1)
table.loc[-1] = table.sum(axis=0)

# قرار گرفتن سطر -1 در انتهای فایل
table = table.sort_index(key=lambda idx: idx.map(lambda x: 9999999 if x == -1 else x))

# ذخیره خروجی در پوشه m5out
table.to_csv(OUTPUT_FILE)

print(f"\nSuccessfully generated table for all {total_switches} context switches: {OUTPUT_FILE}")
print("\nPreview (First 5 switches):")
print(table.iloc[:5, :5])
