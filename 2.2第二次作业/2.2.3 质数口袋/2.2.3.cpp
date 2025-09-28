#include <iostream>
#include <cmath>
using namespace std;

bool pd(int n) {              //判断质数
    if (n <= 1) return false;
    if (n == 2) return true;
    if (n % 2 == 0) return false;

    for (int i = 3; i <= sqrt(n); i += 2) {   //如果一个数不是质数，那么它一定有一个小于或等于其平方根的因子。
        if (n % i == 0) {
            return false;
        }
    }
    return true;
}

int main() {
    int L, a = 2;
    long long sum = 0;
    int num = 0;

    cin >> L;

	while (true) {       //找质数,大于L时停止
        if (pd(a)) {
            if (sum + a > L) {
                break;
            }
            cout << a <<endl;
            sum += a;
            num++;
        }
        a++;
    }

    cout << num ;

    return 0;
}