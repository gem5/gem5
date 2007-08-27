
  SYSTEM TYPE...
  __ZTC__                := False 
  __UNIX__               := True 
  __RISC__               := True 
  SPEC_CPU2000_LP64        := True 
  __MAC__                := False 
  __BCC__                := False 
  __BORLANDC__           := False 
  __GUI__                := False 
  __WTC__                := False 
  __HP__                 := False 

  CODE OPTIONS...
  __MACROIZE_HM__        := True 
  __MACROIZE_MEM__       := True 
  ENV01                  := True 
  USE_HPP_STYPE_HDRS     := False 
  USE_H_STYPE_HDRS       := False 

  CODE INCLUSION PARAMETERS...
  INCLUDE_ALL_CODE       := False 
  INCLUDE_DELETE_CODE    := True 
  __SWAP_GRP_POS__       := True 
  __INCLUDE_MTRX__       := False 
  __BAD_CODE__           := False 
  API_INCLUDE            := False 
  BE_CAREFUL             := False 
  OLDWAY                 := False 
  NOTUSED                := False 

  SYSTEM PARAMETERS...
  EXT_ENUM               := 999999999L 
  CHUNK_CONSTANT         := 55555555 
  CORE_CONSTANT          := 55555555 
  CORE_LIMIT             := 20971520 
  CorePage_Size          := 384000 
  ALIGN_BYTES            := True 
  CORE_BLOCK_ALIGN       :=    8 
  FAR_MEM                := False 

  MEMORY MANAGEMENT PARAMETERS...
  SYSTEM_ALLOC           := True 
  SYSTEM_FREESTORE       := True 
  __NO_DISKCACHE__       := False 
  __FREEZE_VCHUNKS__     := True 
  __FREEZE_GRP_PACKETS__ := True 
  __MINIMIZE_TREE_CACHE__:= True 

  SYSTEM STD PARAMETERS...
  __STDOUT__             := False 
  NULL                   :=    0 
  LPTR                   := False 
  False_Status           :=    1 
  True_Status            :=    0 
  LARGE                  := True 
  TWOBYTE_BOOL           := False 
  __NOSTR__              := False 

  MEMORY VALIDATION PARAMETERS...
  CORE_CRC_CHECK         := False 
  VALIDATE_MEM_CHUNKS    := False 

  SYSTEM DEBUG OPTIONS...
  DEBUG                  := False 
  MCSTAT                 := False 
  TRACKBACK              := False 
  FLUSH_FILES            := False 
  DEBUG_CORE0            := False 
  DEBUG_RISC             := False 
  __TREE_BUG__           := False 
  __TRACK_FILE_READS__   := False 
  PAGE_SPACE             := False 
  LEAVE_NO_TRACE         := True 
  NULL_TRACE_STRS        := False 

  TIME PARAMETERS...
  CLOCK_IS_LONG          := False 
  __DISPLAY_TIME__       := False 
  __TREE_TIME__          := False 
  __DISPLAY_ERRORS__     := False 

  API MACROS...
  __BMT01__              := True 
  OPTIMIZE               := True 

  END OF DEFINES.



              ...   IMPLODE MEMORY ...

  SWAP to DiskCache := False

  FREEZE_GRP_PACKETS:= True

  QueBug            := 1000

  sizeof(boolean)      =  4
  sizeof(sizetype)     =  4
  sizeof(chunkstruc)   = 32

  sizeof(shorttype )   =  2
  sizeof(idtype    )   =  2
  sizeof(sizetype  )   =  4
  sizeof(indextype )   =  4
  sizeof(numtype   )   =  4
  sizeof(handletype)   =  4
  sizeof(tokentype )   =  8

  sizeof(short     )   =  2
  sizeof(int       )   =  4

  sizeof(lt64      )   =  4
  sizeof(farlongtype)  =  4
  sizeof(long      )   =  8
  sizeof(longaddr  )   =  8

  sizeof(float     )   =  4
  sizeof(double    )   =  8

  sizeof(addrtype  )   =  8
  sizeof(char *    )   =  8
 ALLOC   CORE_1    :: 16
 BHOOLE NATH

 OPEN File ./input/lendian.rnv 
    *Status            =   0
   DB HDR restored from FileVbn[  0]
    DB BlkDirOffset      : @  2030c0
    DB BlkDirChunk       : Chunk[  10] AT Vbn[3146]
    DB BlkTknChunk       : Chunk[  11] AT Vbn[3147]
    DB BlkSizeChunk      : Chunk[  12] AT Vbn[3148]
 DB Handle Chunk's StackPtr = 20797

  DB[ 1] LOADED;  Handles= 20797
 KERNEL in CORE[ 1] Restored @ 4005c800

 OPEN File ./input/lendian.wnv 
    *Status            =   0
   DB HDR restored from FileVbn[  0]
    DB BlkDirOffset      : @   21c40
    DB BlkDirChunk       : Chunk[  31] AT Vbn[ 81]
    DB BlkTknChunk       : Chunk[  32] AT Vbn[ 82]
    DB BlkSizeChunk      : Chunk[  33] AT Vbn[ 83]
 DB Handle Chunk's StackPtr = 17

  DB[ 2] LOADED;  Handles= 17
 VORTEx_Status == -8 || fffffff8

    BE HERE NOW !!!



               ... VORTEx ON LINE ...


              ...   END OF SESSION ...
