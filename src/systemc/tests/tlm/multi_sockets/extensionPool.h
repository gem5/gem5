/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

#ifndef __EXTENSION_POOL_H__
#define __EXTENSION_POOL_H__

template <class T>
class ExtensionPool{
  struct entry{
    public:
    entry(T* content){
      that=content;
      next=NULL;
    }
    T* that;
    entry* next;
  };
  
public:
  ExtensionPool(int size): used(NULL){
    unused=new entry(new T());  //create first one
    mine.push_back(unused->that);
    for (int i=0; i<size-1; i++){
      entry* e=new entry(new T());
      e->next=unused;
      unused=e;
      mine.push_back(unused->that);
    }
  }
  
  ~ExtensionPool(){
    //delete all T* that belong to this pool
    for (unsigned int i=0; i<mine.size(); i++){
      delete mine[i];
    }
    
    //delete all unused elements
    while (unused){
      entry* e=unused;
      unused=unused->next;
      delete e;
    }

    //delete all used elements
    while (used){
      entry* e=used;
      used=used->next;
      delete e;
    }
  }
  
  bool is_from(T* cont){ //slow!!!
    for (int i=0; i<mine.size(); i++){
      if (mine[i]==cont) return true;
    }
    return false;
  }
  
  T* construct(){
    entry* e;
    if (unused==NULL){
      e=new entry(new T());
      mine.push_back(e->that);
    }
    else{
      e=unused;
      unused=unused->next;
    }
    e->next=used;
    used=e;
    return used->that; //if all elements of pool are used, just create a new one and go on      
  }

  void free (T* cont){
    sc_assert(used);
    entry* e=used;
    used=e->next;
    e->that=cont;
    e->next=unused;
    unused=e;
  }
  
private:
  entry* unused;
  entry* used;
  std::vector<T*> mine; //just for clean up and is_from
};

#endif

