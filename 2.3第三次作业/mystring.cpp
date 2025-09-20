#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_set>
#include <sstream>
using namespace std;

int main() {
    int n;
    cin >> n;
    cin.ignore(); // 忽略换行符
    
    // 创建10000个序列
    vector<vector<int>> seqs(10000);
    
    for (int i = 0; i < n; i++) {
        string line;
        getline(cin, line);
        stringstream ss(line);
        string command;
        ss >> command;
        
        if (command == "new") {
            int id;
            ss >> id;
            seqs[id] = vector<int>(); // 清空序列
        } 
        else if (command == "add") {
            int id, num;
            ss >> id >> num;
            seqs[id].push_back(num);
        } 
        else if (command == "merge") {
            int id1, id2;
            ss >> id1 >> id2;
            if (id1 != id2) {
                // 将id2的元素添加到id1
                seqs[id1].insert(seqs[id1].end(), seqs[id2].begin(), seqs[id2].end());
                seqs[id2].clear(); // 清空id2
            }
        } 
        else if (command == "unique") {
            int id;
            ss >> id;
            // 使用unordered_set去重
            unordered_set<int> unique_set(seqs[id].begin(), seqs[id].end());
            seqs[id].assign(unique_set.begin(), unique_set.end());
        } 
        else if (command == "out") {
            int id;
            ss >> id;
            // 排序并输出
            sort(seqs[id].begin(), seqs[id].end());
            for (size_t j = 0; j < seqs[id].size(); j++) {
                if (j != 0) cout << " ";
                cout << seqs[id][j];
            }
            cout << endl;
        }
    }
    
    return 0;
}
