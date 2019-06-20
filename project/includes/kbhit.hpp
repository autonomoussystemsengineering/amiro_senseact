/* This function checks if some button was pressed in the console.
 * Here is a little example:
 * 
 * while(true) {
 * if (kbhit())
      {
           KB_code = getchar();
           std::cout << "KB_code = " << KB_code 
           switch (KB_code)
           {
               case KB_SPACE:
                   std::cout << "Pressed Space" << std::endl;
               break;
               case KB_ENTER:
                   std::cout << "Pressed Space" << std::endl;
               break;
           }
      }
   }
 */


#define KB_UP 119
#define KB_DOWN 115
#define KB_LEFT 97
#define KB_RIGHT 100
#define KB_SPACE 32
#define KB_ESCAPE 27
#define KB_ESC 27
#define KB_ENTER 10
#define KB_Q 113
#define KB_W 119
#define KB_E 101
#define KB_A 97
#define KB_S 115
#define KB_Y 121
#define KB_X 120
#define KB_D 100
#define KB_C 99

#define KB_1 49
#define KB_2 50
#define KB_3 51
#define KB_4 52
#define KB_5 53
#define KB_6 54
#define KB_7 55
#define KB_8 56
#define KB_9 57
#define KB_0 48


#include <curses.h>
#include <termios.h>
// #include <unistd.h>
#include <fcntl.h>


 
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}