#include<iostream>
#include<vector>
using namespace std;

void px(int sz[], int n) {             //冒泡排序
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

    vector<int> num(a);  // 使用vector作为动态数组

    for (int i = 0; i < a; i++) {
        cin >> num[i];
    }

    px(num.data(), a);  // 使用data()获取底层数组指针

    for (int i = 0; i < a; i++) {
        cout << num[i] << " ";
    }

    return 0;
}