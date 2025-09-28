#include <iostream>
#include <vector>
using namespace std;

long long fb(int n) {             // 计算斐波那契数列的第n项
    if (n <= 1) {
        return n;
    }
    long long x = 0, y = 1, z;
    for (int i = 2; i <= n; i++) {
        z = x + y;
        x = y;
        y = z;
    }
    return z;
}

int main() {
    int a;
    cin >> a;
	vector<int> num(a);        //动态数组

    for (int i = 0; i < a; i++) {
        cin >> num[i];
    }

	for (int i = 0; i < a; i++) {        //输出结果
        cout << fb(num[i]) << endl;
    }

    return 0;
}