import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# بارگذاری داده‌ها
print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

if df.empty:
    print("CSV is empty!")
    exit()

# جدا کردن Context Switch های مختلف بر اساس پرش زمانی
gap_threshold = 10_000_000 
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['New_Window'] = df['Tick_Diff'] > gap_threshold
df['Window_ID'] = df['New_Window'].cumsum()

# انتخاب یک Context Switch خاص برای رسم (مثلاً Window شماره 1)
TARGET_WINDOW = 1
window_data = df[df['Window_ID'] == TARGET_WINDOW].copy()

if window_data.empty:
    print(f"Window {TARGET_WINDOW} not found.")
    exit()

# نرمال‌سازی زمان تا نمودار از زمان صفر شروع شود
window_data['Relative_Time_ns'] = (window_data['Tick'] - window_data['Tick'].min()) / 1000.0

# رسم نمودار پراکندگی (Scatter Plot)
plt.figure(figsize=(14, 7))

# رسم نقاط: محور x زمان است و محور y آدرس داده (VAddr)
plt.scatter(window_data['Relative_Time_ns'], window_data['VAddr'], 
            alpha=0.6, s=10, color='crimson', marker='o')

# تنظیمات نمودار
plt.title(f'Memory Address Space Access Over Time (Context Switch #{TARGET_WINDOW})', fontsize=14)
plt.xlabel('Time since Context Switch Started (Nanoseconds)', fontsize=12)
plt.ylabel('Data Address Space (Virtual Address)', fontsize=12)

# تبدیل اعداد محور y به فرمت هگزادسیمال (Hex) تا خواندن آدرس‌ها راحت باشد
formatter = ticker.FuncFormatter(lambda x, pos: f'0x{int(x):X}')
plt.gca().yaxis.set_major_formatter(formatter)

plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()

plt.savefig('context_switch_address_scatter.png', dpi=300)
print("Scatter plot saved as 'context_switch_address_scatter.png'")
