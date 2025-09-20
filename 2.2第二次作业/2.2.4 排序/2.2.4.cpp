#include<iostream>
using namespace std;

void px(int sz[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int m = 0; m < n - i - 1; m++) {
            if (sz[m] > sz[m + 1]) {
                int temp = sz[m];
                sz[m] = sz[m + 1];
                sz[m + 1] = temp;
            }
        }
    }
}

int main() {
    int a;
    cin >> a;

    int* num = new int[a];

    for (int i = 0; i < a; i++) {
        cin >> num[i];
    }

    px(num, a);

    for (int i = 0; i < a; i++) {
        cout << num[i] << " ";
    }

    delete[] num;

    return 0;
}