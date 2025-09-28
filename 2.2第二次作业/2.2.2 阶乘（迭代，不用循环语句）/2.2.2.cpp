#include <iostream>
using namespace std;
long long jc(int n) {
	if (n <= 1) {
		return 1;
	}
	else
	{
		return n * jc(n - 1);         //µü´ú¼ÆËã
	}
}
int main() {
	int a;
	cin >> a;
	cout << jc(a) << endl;
	return 0;
}