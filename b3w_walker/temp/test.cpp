#include <iostream>

class aaa{
public:
  virtual bool tmp() =0;
};

class bbb: aaa{
public:
  bool tmp(){
    return true;
  }
};

class ccc: aaa{
};

int main(void){
  bbb* bb =new ccc;
  std::cout<<bb->tmp()<<std::endl; 
  return 0;
}
