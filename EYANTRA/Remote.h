

#define Range 50
#define Centre(x,y) y>(-Range) && y<Range && x>(-Range) && x<Range
#define Backward(x,y) y>=x && y>=(-x)       //mapping x and y recieved to zones
#define Forward(x,y) y<=x && y<=(-x)
#define Right(x,y) y<x && y>(-x)
#define Left(x,y) y>x && y<(-x)


void remote( int &x, int &y, int &MagF,int &MagB,int &present_val,int &mode)
{
  if(Serial1.available()>17){
    if(Serial1.read()==0x7E){
      for(int i=1;i<12;i++){
        byte waste=Serial1.read();
      }   
      
      int tog=Serial1.read();
      int x_msb=Serial1.read();
      int x_lsb=Serial1.read();
      int y_msb=Serial1.read();
      int y_lsb=Serial1.read();
     
      int x1 = (x_msb*256) + x_lsb;
      int y1 = (y_msb*256) + y_lsb; 
      x=map(x1,0,1023,-255,255);// analog write has resolution of 255
      y=map(y1,0,1023,-255,255);
      
      MagF=(tog>>3) & 00000001;
      MagB=(tog) & 00000001;
      present_val=(tog>>5) & 00000001;
      mode=(tog>>7) & 00000001;               
      }
    } 
}
