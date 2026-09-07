import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# تنظیمات ظاهری نمودارها برای گزارش‌های آکادمیک
plt.rcParams.update({
    'font.size': 12,
    'axes.labelsize': 14,
    'axes.titlesize': 15,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'figure.titlesize': 16,
    'figure.autolayout': True
})
sns.set_theme(style="whitegrid", palette="deep")
OUTPUT_DIR = "analysis_plots"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ==============================================================================
# ۱. بارگذاری و تمیزسازی داده‌ها
# ==============================================================================
print("[-] در حال بارگذاری داده‌های شبیه‌سازی...")

# الف) فواصل زمانی کانتکست سوییچ
df_intervals = pd.DataFrame()
if os.path.exists("m5out/ctx_switch_intervals.csv") and os.path.getsize("m5out/ctx_switch_intervals.csv") > 0:
    df_intervals = pd.read_csv("m5out/ctx_switch_intervals.csv", header=None,
                               names=["begin_tick", "end_tick", "prev_pid", "next_pid"])
    df_intervals["duration_ns"] = (df_intervals["end_tick"] - df_intervals["begin_tick"]) / 1000.0
    print(f" [+] تعداد کانتکست سوییچ‌های ثبت‌شده: {len(df_intervals)}")

# ب) ردپای دستورات کانتکست سوییچ
df_inst = pd.DataFrame()
if os.path.exists("m5out/ctx_inst_trace.csv") and os.path.getsize("m5out/ctx_inst_trace.csv") > 0:
    df_inst = pd.read_csv("m5out/ctx_inst_trace.csv", header=None,
                          names=["tick", "core_id", "pc", "mnemonic"])
    df_inst["tick"] = pd.to_numeric(df_inst["tick"], errors="coerce")
    df_inst = df_inst.dropna().sort_values("tick")
    print(f" [+] تعداد کل دستورات ثبت‌شده داخل سوییچ: {len(df_inst)}")

# ج) خطاهای کش دستور (L1-I Misses)
df_misses = pd.DataFrame()
if os.path.exists("m5out/ctx_icache_misses.csv") and os.path.getsize("m5out/ctx_icache_misses.csv") > 0:
    df_misses = pd.read_csv("m5out/ctx_icache_misses.csv", header=None,
                            names=["tick", "core_id", "miss_vaddr"])
    df_misses["tick"] = pd.to_numeric(df_misses["tick"], errors="coerce")
    df_misses = df_misses.dropna().sort_values("tick")
    print(f" [+] تعداد خطاهای L1-I ثبت‌شده: {len(df_misses)}")

# ==============================================================================
# ۲. نمودار ۱: توزیع فراوانی دستورات اسمبلی داخل Context Switch
# ==============================================================================
if not df_inst.empty:
    print("[-] در حال رسم نمودار توزیع دستورات...")
    plt.figure(figsize=(10, 6))
    top_inst = df_inst["mnemonic"].value_counts().head(12)
    
    ax = sns.barplot(x=top_inst.index, y=top_inst.values, palette="crest")
    plt.title("Instruction Frequency Breakdown during Context Switches", weight="bold")
    plt.xlabel("Instruction Mnemonic (ARM64)")
    plt.ylabel("Execution Count")
    plt.xticks(rotation=45)
    
    for p in ax.patches:
        ax.annotate(f'{int(p.get_height())}', 
                    (p.get_x() + p.get_width() / 2., p.get_height()), 
                    ha='center', va='bottom', fontsize=10, xytext=(0, 3), 
                    textcoords='offset points')
                    
    plt.savefig(f"{OUTPUT_DIR}/plot1_instruction_distribution.png", dpi=300)
    plt.close()

# ==============================================================================
# ۳. تحلیل و استخراج ماتریس همبستگی EPI (Entangling Prefetching)
# ==============================================================================
if not df_inst.empty and not df_misses.empty:
    print("[-] در حال محاسبه همبستگی‌های EPI (محرک -> هدف)...")
    
    # پنجره زمانی بررسی همبستگی (مثلاً 5000 سیکل = 2,500,000 تیک در پردازنده 2GHz)
    DELTA_TICK_WINDOW = 2500000 
    correlations = []

    for core in df_inst["core_id"].unique():
        core_insts = df_inst[df_inst["core_id"] == core]
        core_misses = df_misses[df_misses["core_id"] == core]

        for _, miss in core_misses.iterrows():
            m_tick = miss["tick"]
            m_addr = miss["miss_vaddr"]

            # پیدا کردن دستوراتی که در بازه مشخص قبل از این Miss اجرا شده‌اند
            prior_insts = core_insts[(core_insts["tick"] <= m_tick) & 
                                     (core_insts["tick"] >= m_tick - DELTA_TICK_WINDOW)]
            
            if not prior_insts.empty:
                # آخرین دستور اجرا شده قبل از خطا به عنوان Trigger بالقوه
                last_trigger = prior_insts.iloc[-1]
                latency_ticks = m_tick - last_trigger["tick"]
                correlations.append({
                    "core_id": core,
                    "trigger_pc": last_trigger["pc"],
                    "trigger_mnemonic": last_trigger["mnemonic"],
                    "miss_vaddr": m_addr,
                    "latency_ticks": latency_ticks
                })

    df_epi = pd.DataFrame(correlations)
    
    if not df_epi.empty:
        # فیلتر جفت‌های با تکرار بالا
        pair_counts = df_epi.groupby(["trigger_pc", "trigger_mnemonic", "miss_vaddr"]).size().reset_index(name="entangle_count")
        pair_counts = pair_counts.sort_values(by="entangle_count", ascending=False)
        pair_counts.to_csv(f"{OUTPUT_DIR}/epi_entangled_pairs.csv", index=False)
        
        # رسم نمودار همبستگی EPI
        plt.figure(figsize=(12, 7))
        top_pairs = pair_counts.head(10).copy()
        top_pairs["pair_label"] = top_pairs["trigger_mnemonic"] + " (" + top_pairs["trigger_pc"].str[-6:] + ") -> " + top_pairs["miss_vaddr"].str[-6:]
        
        sns.barplot(data=top_pairs, x="entangle_count", y="pair_label", palette="mako")
        plt.title("Top Entangled Pairs (EPI: Trigger Inst -> Miss Target)", weight="bold")
        plt.xlabel("Correlation / Recurrence Count")
        plt.ylabel("Entangled Pair: Trigger (PC) -> Target Miss (Cache Line)")
        plt.savefig(f"{OUTPUT_DIR}/plot2_epi_correlation_ranking.png", dpi=300)
        plt.close()

# ==============================================================================
# ۴. نمودار ۳: تأخیر زمانی (Latency) کانتکست سوییچ‌ها بر حسب نانوثانیه
# ==============================================================================
if not df_intervals.empty:
    print("[-] در حال رسم نمودار Latency کانتکست سوییچ...")
    plt.figure(figsize=(9, 5))
    
    # تفکیک سوییچ‌های به Idle (PID 0) و سوییچ‌های برنامه‌ها
    df_intervals["switch_type"] = np.where((df_intervals["prev_pid"] == 0) | (df_intervals["next_pid"] == 0), 
                                           "Kernel/Idle (PID 0)", "User App Switch")
    
    sns.boxplot(data=df_intervals, x="switch_type", y="duration_ns", palette="Set2", width=0.4)
    sns.stripplot(data=df_intervals, x="switch_type", y="duration_ns", color="black", alpha=0.5, jitter=0.2)
    
    plt.title("Context Switch Execution Latency Distribution", weight="bold")
    plt.xlabel("Switch Category")
    plt.ylabel("Duration (Nanoseconds)")
    plt.savefig(f"{OUTPUT_DIR}/plot3_context_switch_latency.png", dpi=300)
    plt.close()

# ==============================================================================
# ۵. نمودار ۴: تایم‌لاین میکروسکوپی یک نمونه Context Switch (Execution Trace)
# ==============================================================================
if not df_inst.empty:
    print("[-] در حال رسم تایم‌لاین جزئی اجرای یک سوییچ...")
    sample_core = df_inst["core_id"].iloc[0]
    sample_inst = df_inst[df_inst["core_id"] == sample_core].head(60)
    
    plt.figure(figsize=(14, 5))
    plt.plot(sample_inst["tick"], range(len(sample_inst)), marker='o', linestyle='-', color='#1f77b4', label="Instruction Commit")
    
    # اضافه کردن نام دستورات به نقاط
    for i, row in sample_inst.reset_index().iterrows():
        if i % 2 == 0: # برای جلوگیری از شلوغی متن
            plt.text(row["tick"], i, f" {row['mnemonic']}", fontsize=8, verticalalignment='bottom')

    # اگر در همین بازه خطای کش رخ داده بود نمایش بده
    if not df_misses.empty:
        core_misses = df_misses[(df_misses["core_id"] == sample_core) & 
                                (df_misses["tick"] >= sample_inst["tick"].min()) & 
                                (df_misses["tick"] <= sample_inst["tick"].max())]
        for _, m in core_misses.iterrows():
            plt.axvline(x=m["tick"], color='red', linestyle='--', alpha=0.7, label="L1-I Cache Miss")
            
    plt.title(f"Micro-Timeline of Executed Instructions on Core {sample_core}", weight="bold")
    plt.xlabel("Simulation Tick (ps)")
    plt.ylabel("Instruction Sequence Index")
    plt.savefig(f"{OUTPUT_DIR}/plot4_micro_timeline_trace.png", dpi=300)
    plt.close()

print(f"\n[✓] تمام نمودارها و جداول تحلیل در پوشه '{OUTPUT_DIR}' با کیفیت 300 DPI ذخیره شدند.")
