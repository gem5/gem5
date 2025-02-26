#include <iostream>
#include <vector>
using namespace std;

int main() {
    int n = 100000;
    vector<int> arr(n);
    for (int i = 0; i < n; i++) {
        arr[i] = i * i;
    }
    return 0;
}