#include<iostream>
#include<vector>
using namespace std;

void px(int sz[], int n) {             //ð������,����sz������,ʵ����ָ��
    for (int i = 0; i < n - 1; i++) {           // ���ѭ����������
        for (int m = 0; m < n - i - 1; m++) {  // �ڲ�ѭ���������ڱȽ�
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

    vector<int> num(a);  // ʹ��vector��Ϊ��̬����

    for (int i = 0; i < a; i++) {
        cin >> num[i];
    }

	px(num.data(), a);  // ʹ��data()��ȡ����ָ�룬ֱ�Ӵ��뺯��

    for (int i = 0; i < a; i++) {
        cout << num[i] << " ";
    }

    return 0;
}