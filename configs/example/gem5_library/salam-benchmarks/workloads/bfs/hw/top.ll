; ModuleID = 'top.c'
source_filename = "top.c"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "armv7-pc-none-eabi"

; Function Attrs: nofree norecurse nounwind
define dso_local void @top(i64 noundef %0, i64 noundef %1, i64 noundef %2, i64 noundef %3, i32 noundef %4) local_unnamed_addr #0 {
  store volatile i64 %0, i64* inttoptr (i32 788529153 to i64*), align 8, !tbaa !8
  store volatile i64 788529344, i64* inttoptr (i32 788529161 to i64*), align 8, !tbaa !8
  store volatile i32 2048, i32* inttoptr (i32 788529169 to i32*), align 4, !tbaa !12
  store volatile i8 1, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  br label %6

6:                                                ; preds = %6, %5
  %7 = load volatile i8, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  %8 = and i8 %7, 4
  %9 = icmp eq i8 %8, 0
  br i1 %9, label %6, label %10, !llvm.loop !15

10:                                               ; preds = %6
  store volatile i64 %1, i64* inttoptr (i32 788529153 to i64*), align 8, !tbaa !8
  store volatile i64 788531456, i64* inttoptr (i32 788529161 to i64*), align 8, !tbaa !8
  store volatile i32 16384, i32* inttoptr (i32 788529169 to i32*), align 4, !tbaa !12
  store volatile i8 1, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  br label %11

11:                                               ; preds = %11, %10
  %12 = load volatile i8, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  %13 = and i8 %12, 4
  %14 = icmp eq i8 %13, 0
  br i1 %14, label %11, label %15, !llvm.loop !18

15:                                               ; preds = %11
  store volatile i64 %2, i64* inttoptr (i32 788529153 to i64*), align 8, !tbaa !8
  store volatile i64 788547904, i64* inttoptr (i32 788529161 to i64*), align 8, !tbaa !8
  store volatile i32 256, i32* inttoptr (i32 788529169 to i32*), align 4, !tbaa !12
  store volatile i8 1, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  br label %16

16:                                               ; preds = %16, %15
  %17 = load volatile i8, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  %18 = and i8 %17, 4
  %19 = icmp eq i8 %18, 0
  br i1 %19, label %16, label %20, !llvm.loop !19

20:                                               ; preds = %16
  store i32 %4, i32* inttoptr (i32 788529281 to i32*), align 4, !tbaa !12
  store volatile i8 1, i8* inttoptr (i32 788529280 to i8*), align 128, !tbaa !14
  br label %21

21:                                               ; preds = %21, %20
  %22 = load volatile i8, i8* inttoptr (i32 788529280 to i8*), align 128, !tbaa !14
  %23 = and i8 %22, 4
  %24 = icmp eq i8 %23, 0
  br i1 %24, label %21, label %25, !llvm.loop !20

25:                                               ; preds = %21
  store volatile i64 788548224, i64* inttoptr (i32 788529153 to i64*), align 8, !tbaa !8
  store volatile i64 %3, i64* inttoptr (i32 788529161 to i64*), align 8, !tbaa !8
  store volatile i32 40, i32* inttoptr (i32 788529169 to i32*), align 4, !tbaa !12
  store volatile i8 1, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  br label %26

26:                                               ; preds = %26, %25
  %27 = load volatile i8, i8* inttoptr (i32 788529152 to i8*), align 16777216, !tbaa !14
  %28 = and i8 %27, 4
  %29 = icmp eq i8 %28, 0
  br i1 %29, label %26, label %30, !llvm.loop !21

30:                                               ; preds = %26
  ret void
}

attributes #0 = { nofree norecurse nounwind "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="generic" "target-features"="+armv7-a,+dsp,+soft-float,+strict-align,-aes,-bf16,-d32,-dotprod,-fp-armv8,-fp-armv8d16,-fp-armv8d16sp,-fp-armv8sp,-fp16,-fp16fml,-fp64,-fpregs,-fullfp16,-mve,-mve.fp,-neon,-sha2,-thumb-mode,-vfp2,-vfp2sp,-vfp3,-vfp3d16,-vfp3d16sp,-vfp3sp,-vfp4,-vfp4d16,-vfp4d16sp,-vfp4sp" "use-soft-float"="true" }

!llvm.module.flags = !{!0, !1, !2, !3, !4, !5, !6}
!llvm.ident = !{!7}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 1, !"min_enum_size", i32 4}
!2 = !{i32 1, !"branch-target-enforcement", i32 0}
!3 = !{i32 1, !"sign-return-address", i32 0}
!4 = !{i32 1, !"sign-return-address-all", i32 0}
!5 = !{i32 1, !"sign-return-address-with-bkey", i32 0}
!6 = !{i32 7, !"frame-pointer", i32 2}
!7 = !{!"Ubuntu clang version 14.0.0-1ubuntu1.1"}
!8 = !{!9, !9, i64 0}
!9 = !{!"long long", !10, i64 0}
!10 = !{!"omnipotent char", !11, i64 0}
!11 = !{!"Simple C/C++ TBAA"}
!12 = !{!13, !13, i64 0}
!13 = !{!"int", !10, i64 0}
!14 = !{!10, !10, i64 0}
!15 = distinct !{!15, !16, !17}
!16 = !{!"llvm.loop.mustprogress"}
!17 = !{!"llvm.loop.unroll.disable"}
!18 = distinct !{!18, !16, !17}
!19 = distinct !{!19, !16, !17}
!20 = distinct !{!20, !16, !17}
!21 = distinct !{!21, !16, !17}
