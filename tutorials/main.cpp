
#include <iostream>

// use namespace to have two variables/member functions with the same name
using namespace std;

class robot {

public:
    void sensor();
}obj;

void robot::sensor() {
    cout<<"Detected"<<endl;
}


// Usage: g++ main.cpp -o main
// ./main
int main(int argc, const char * argv[]) {

    obj.sensor();
    cout<<"Some filler text here\n";

    return 0;

}
