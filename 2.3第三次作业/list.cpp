#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

using namespace std;

struct Seq {
    vector<int> data;
    bool sorted;
};

Seq seqArr[10000];

int main() {
    int n;
    cin >> n;
    cin.ignore();
    
    for (int i = 0; i < 10000; i++) {
        seqArr[i].data.clear();
        seqArr[i].sorted = true;
    }
    
    for (int i = 0; i < n; i++) {
        string line;
        getline(cin, line);
        stringstream ss(line);
        string command;
        ss >> command;
        if (command == "new") {
            int id;
            ss >> id;
            seqArr[id].data.clear();
            seqArr[id].sorted = true;
        } else if (command == "add") {
            int id, num;
            ss >> id >> num;
            seqArr[id].data.push_back(num);
            seqArr[id].sorted = false;
        } else if (command == "merge") {
            int id1, id2;
            ss >> id1 >> id2;
            if (id1 == id2) {
                continue;
            }
            if (seqArr[id1].data.size() >= seqArr[id2].data.size()) {
                seqArr[id1].data.insert(seqArr[id1].data.end(), seqArr[id2].data.begin(), seqArr[id2].data.end());
                seqArr[id1].sorted = false;
                seqArr[id2].data.clear();
                seqArr[id2].sorted = true;
            } else {
                seqArr[id2].data.insert(seqArr[id2].data.end(), seqArr[id1].data.begin(), seqArr[id1].data.end());
                seqArr[id2].sorted = false;
                swap(seqArr[id1].data, seqArr[id2].data);
                swap(seqArr[id1].sorted, seqArr[id2].sorted);
                seqArr[id2].data.clear();
                seqArr[id2].sorted = true;
            }
        } else if (command == "unique") {
            int id;
            ss >> id;
            if (!seqArr[id].sorted) {
                sort(seqArr[id].data.begin(), seqArr[id].data.end());
                seqArr[id].sorted = true;
            }
            vector<int>& v = seqArr[id].data;
            v.erase(unique(v.begin(), v.end()), v.end());
        } else if (command == "out") {
            int id;
            ss >> id;
            if (!seqArr[id].sorted) {
                sort(seqArr[id].data.begin(), seqArr[id].data.end());
                seqArr[id].sorted = true;
            }
            for (size_t j = 0; j < seqArr[id].data.size(); j++) {
                if (j != 0) {
                    cout << " ";
                }
                cout << seqArr[id].data[j];
            }
            cout << endl;
        }
    }
    return 0;
}
