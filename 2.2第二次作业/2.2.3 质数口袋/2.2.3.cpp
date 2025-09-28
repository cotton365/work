#include <iostream>
#include <cmath>
using namespace std;

bool pd(int n) {              //�ж�����
    if (n <= 1) return false;
    if (n == 2) return true;
    if (n % 2 == 0) return false;

    for (int i = 3; i <= sqrt(n); i += 2) {   //���һ����������������ô��һ����һ��С�ڻ������ƽ���������ӡ�
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

	while (true) {       //������,����Lʱֹͣ
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