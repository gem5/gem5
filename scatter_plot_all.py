import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# بارگذاری داده‌ها (با ۴ ستون)
print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'CoreID', 'PC', 'VAddr'])

if df.empty:
    print("CSV is empty!")
    exit()

# جدا کردن Context Switch های مختلف بر اساس پرش زمانی بزرگ
gap_threshold = 10_000_000 
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['New_Window'] = df['Tick_Diff'] > gap_threshold
df['Window_ID'] = df['New_Window'].cumsum()

num_switches = df['Window_ID'].nunique()
print(f"Total Context Switches found: {num_switches}")

# محاسبه زمان نسبی (از صفر) برای *هر* Context Switch به صورت مجزا
# این کار باعث می‌شود همه جابجایی‌ها از زمان 0 شروع شوند و روی هم بیفتند
df['Relative_Time_ns'] = df.groupby('Window_ID')['Tick'].transform(lambda x: (x - x.min()) / 1000.0)

# رسم نمودار پراکندگی (Scatter Plot)
plt.figure(figsize=(14, 7))

# استفاده از alpha بسیار پایین (0.05) باعث می‌شود نقاط پرتراکم، پررنگ‌تر دیده شوند
plt.scatter(df['Relative_Time_ns'], df['VAddr'], 
            alpha=0.05, s=2, color='crimson', marker='o')

# تنظیمات نمودار
plt.title(f'Memory Access Signature: {num_switches} Context Switches Overlaid', fontsize=14)
plt.xlabel('Relative Time from Start of Context Switch (Nanoseconds)', fontsize=12)
plt.ylabel('Data Address Space (Virtual Address)', fontsize=12)

# تبدیل اعداد محور y به فرمت هگزادسیمال (Hex)
formatter = ticker.FuncFormatter(lambda x, pos: f'0x{int(x):X}')
plt.gca().yaxis.set_major_formatter(formatter)

plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()

plt.savefig('all_context_switches_scatter.png', dpi=300, facecolor='white')
print("Scatter plot saved as 'all_context_switches_scatter.png'")
