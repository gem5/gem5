import os
import pandas as pd

print("[-] در حال بارگذاری داده‌ها برای ساخت جدول دقیق دستورات...")

# ۱. بارگذاری بازه‌های سوییچ‌ها
df_intervals = pd.read_csv(
    "m5out/ctx_switch_intervals.csv", 
    header=None,
    names=["begin_tick", "end_tick", "prev_pid", "next_pid"]
)
df_intervals["switch_id"] = range(1, len(df_intervals) + 1)
df_intervals = df_intervals.sort_values("begin_tick")

# ۲. بارگذاری دستورات داخل سوییچ
df_inst = pd.read_csv(
    "m5out/ctx_inst_trace.csv", 
    header=None,
    names=["tick", "core_id", "pc", "mnemonic"]
)
df_inst["tick"] = pd.to_numeric(df_inst["tick"], errors="coerce")
df_inst = df_inst.dropna().sort_values("tick")

# ۳. نگاشت دقیق هر دستور به شماره کانتکست سوییچ مربوطه
print("[-] در حال انتساب دستورات به سوییچ‌ها...")
df_merged = pd.merge_asof(
    df_inst, 
    df_intervals[["begin_tick", "end_tick", "switch_id"]], 
    left_on="tick", 
    right_on="begin_tick", 
    direction="backward"
)

# فیلتر دستوراتی که دقیقاً درون بازه زمانی سوییچ قرار دارند
df_merged = df_merged[df_merged["tick"] <= df_merged["end_tick"]]

# ۴. ساخت جدول دقیق فراوانی (شمارش تعداد تکرار هر PC در هر کانتکست سوییچ)
print("[-] در حال تجمیع و ساخت instruction_usage_table_exact.csv...")
exact_table = df_merged.groupby(["switch_id", "pc", "mnemonic"]).size().reset_index(name="execution_count")

# استانداردسازی برچسب دستور برای محور Y
exact_table["inst_label"] = "Inst_" + exact_table["pc"].astype(str).str.upper()

# ذخیره خروجی
exact_table.to_csv("m5out/instruction_usage_table_exact.csv", index=False)
print("[✓] فایل m5out/instruction_usage_table_exact.csv با موفقیت ایجاد شد.")
