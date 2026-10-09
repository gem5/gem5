; ModuleID = 'bfs.c'
source_filename = "bfs.c"
target datalayout = "e-m:e-p:32:32-Fi8-i64:64-v128:64:128-a:0:32-n32-S64"
target triple = "armv7-pc-none-eabi"

%struct.node_t_struct = type { i32, i32 }
%struct.edge_t_struct = type { i32 }

; Function Attrs: nofree norecurse nounwind
define dso_local void @bfs(i32 noundef %0) local_unnamed_addr #0 {
  %2 = getelementptr inbounds i8, i8* inttoptr (i32 788547904 to i8*), i32 %0
  store volatile i8 0, i8* %2, align 1, !tbaa !8
  store volatile i32 1, i32* inttoptr (i32 788548224 to i32*), align 128, !tbaa !11
  br label %3

3:                                                ; preds = %38, %1
  %4 = phi i32 [ 0, %1 ], [ %5, %38 ]
  %5 = add nuw nsw i32 %4, 1
  %6 = trunc i32 %5 to i8
  br label %7

7:                                                ; preds = %3, %34
  %8 = phi i32 [ 0, %3 ], [ %36, %34 ]
  %9 = phi i32 [ 0, %3 ], [ %35, %34 ]
  %10 = getelementptr inbounds i8, i8* inttoptr (i32 788547904 to i8*), i32 %8
  %11 = load volatile i8, i8* %10, align 1, !tbaa !8
  %12 = zext i8 %11 to i32
  %13 = icmp eq i32 %4, %12
  br i1 %13, label %14, label %34

14:                                               ; preds = %7
  %15 = getelementptr inbounds %struct.node_t_struct, %struct.node_t_struct* inttoptr (i32 788529344 to %struct.node_t_struct*), i32 %8, i32 0
  %16 = load volatile i32, i32* %15, align 8, !tbaa !13
  %17 = getelementptr inbounds %struct.node_t_struct, %struct.node_t_struct* inttoptr (i32 788529344 to %struct.node_t_struct*), i32 %8, i32 1
  %18 = load volatile i32, i32* %17, align 4, !tbaa !15
  %19 = icmp ult i32 %16, %18
  br i1 %19, label %20, label %34

20:                                               ; preds = %14, %30
  %21 = phi i32 [ %32, %30 ], [ %16, %14 ]
  %22 = phi i32 [ %31, %30 ], [ %9, %14 ]
  %23 = getelementptr inbounds %struct.edge_t_struct, %struct.edge_t_struct* inttoptr (i32 788531456 to %struct.edge_t_struct*), i32 %21, i32 0
  %24 = load volatile i32, i32* %23, align 4, !tbaa !16
  %25 = getelementptr inbounds i8, i8* inttoptr (i32 788547904 to i8*), i32 %24
  %26 = load volatile i8, i8* %25, align 1, !tbaa !8
  %27 = icmp eq i8 %26, 127
  br i1 %27, label %28, label %30

28:                                               ; preds = %20
  store volatile i8 %6, i8* %25, align 1, !tbaa !8
  %29 = add i32 %22, 1
  br label %30

30:                                               ; preds = %28, %20
  %31 = phi i32 [ %29, %28 ], [ %22, %20 ]
  %32 = add nuw i32 %21, 1
  %33 = icmp eq i32 %32, %18
  br i1 %33, label %34, label %20, !llvm.loop !18

34:                                               ; preds = %30, %14, %7
  %35 = phi i32 [ %9, %7 ], [ %9, %14 ], [ %31, %30 ]
  %36 = add nuw nsw i32 %8, 1
  %37 = icmp eq i32 %36, 256
  br i1 %37, label %38, label %7, !llvm.loop !21

38:                                               ; preds = %34
  %39 = add nuw nsw i32 %4, 1
  %40 = getelementptr inbounds i32, i32* inttoptr (i32 788548224 to i32*), i32 %39
  store volatile i32 %35, i32* %40, align 4, !tbaa !11
  %41 = icmp eq i32 %35, 0
  %42 = icmp eq i32 %5, 10
  %43 = select i1 %41, i1 true, i1 %42
  br i1 %43, label %44, label %3, !llvm.loop !22

44:                                               ; preds = %38
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
!9 = !{!"omnipotent char", !10, i64 0}
!10 = !{!"Simple C/C++ TBAA"}
!11 = !{!12, !12, i64 0}
!12 = !{!"int", !9, i64 0}
!13 = !{!14, !12, i64 0}
!14 = !{!"node_t_struct", !12, i64 0, !12, i64 4}
!15 = !{!14, !12, i64 4}
!16 = !{!17, !12, i64 0}
!17 = !{!"edge_t_struct", !12, i64 0}
!18 = distinct !{!18, !19, !20}
!19 = !{!"llvm.loop.mustprogress"}
!20 = !{!"llvm.loop.unroll.disable"}
!21 = distinct !{!21, !19, !20}
!22 = distinct !{!22, !19, !20}
