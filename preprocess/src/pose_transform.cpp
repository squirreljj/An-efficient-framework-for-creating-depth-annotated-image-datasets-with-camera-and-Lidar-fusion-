#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pose_transform.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
using namespace Eigen;
using namespace std;
void Stringsplit(const string& str, const string& splits, vector<string>& res)
{
    if (str == "")		
        return;
    string strs = str + splits;
    size_t pos = strs.find(splits);
    int step = splits.size();
    while (pos != strs.npos){
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        strs = strs.substr(pos + step, strs.size());
        pos = strs.find(splits);
    }
}
int main(int argc, char** argv) {
    string yaml_path=argv[1];
    std::ifstream fin(yaml_path);
    YAML::Node node = YAML::Load(fin);
    std::string map_pose_path = node["pose_map_path"].as<std::string>();
    std::string scan_pose_path = node["delta_pose_path"].as<std::string>();
    ifstream file(map_pose_path);
    ofstream  file2(scan_pose_path);
    if (!file.is_open()){
        cerr << "cant load pose file" << endl;
        return -1;
    }
    if (!file2.is_open()){
        cerr << "cant load pose file" << endl;
        return -1;
    }
    string str;
    vector<string> data;
	while(getline(file,str))
    	{		
		vector<string> list_str;
        	Stringsplit(str," ",list_str);
		data.insert(data.end(),list_str.begin(),list_str.end());
	}
    auto isEmptyString = [](const string& str) { return str.empty(); };
    auto new_end = std::remove_if(data.begin(), data.end(), isEmptyString);
    data.erase(new_end, data.end());
    int rows=data.size();
    for(int i=0;i<data.size();i++)
        cout<<data[i]<<" "<<i<<endl;
    cout<<rows<<" "<<rows/13<<endl;
    file2<<data[0]<<" "<<stod(data[1])<<" "<<stod(data[2])<<" "<<stod(data[3])<<" "<<stod(data[4])<<" "<<
        stod(data[5])<<" "<<stod(data[6])<<" "<<stod(data[7])<<" "<<stod(data[8])<<" "<<stod(data[9])<<" "<<
        stod(data[10])<<" "<<stod(data[11])<<" "<<stod(data[12])<<endl;
    for (int i=1;i<(rows/13);i++){
        double r11=stod(data[(i-1)*13+1]);
        double r12=stod(data[(i-1)*13+2]);
        double r13=stod(data[(i-1)*13+3]);
        double r21=stod(data[(i-1)*13+4]);
        double r22=stod(data[(i-1)*13+5]);
        double r23=stod(data[(i-1)*13+6]);
        double r31=stod(data[(i-1)*13+7]);
        double r32=stod(data[(i-1)*13+8]);
        double r33=stod(data[(i-1)*13+9]);
        double t1=stod(data[(i-1)*13+10]);
        double t2=stod(data[(i-1)*13+11]);
        double t3=stod(data[(i-1)*13+12]);

        double _r11=stod(data[i*13+1]);
        double _r12=stod(data[i*13+2]);
        double _r13=stod(data[i*13+3]);
        double _r21=stod(data[i*13+4]);
        double _r22=stod(data[i*13+5]);
        double _r23=stod(data[i*13+6]);
        double _r31=stod(data[i*13+7]);
        double _r32=stod(data[i*13+8]);
        double _r33=stod(data[i*13+9]);
        double _t1=stod(data[i*13+10]);
        double _t2=stod(data[i*13+11]);
        double _t3=stod(data[i*13+12]);
        Matrix3d W_R_L1;
        W_R_L1<<r11,r12,r13,r21,r22,r23,r31,r32,r33;
        Vector3d W_t_L1;
        W_t_L1<<t1,t2,t3;

        Matrix3d W_R_L2;
        W_R_L2<<_r11,_r12,_r13,_r21,_r22,_r23,_r31,_r32,_r33;
        Vector3d W_t_L2;
        W_t_L2<<_t1,_t2,_t3;

        Matrix3d L1_R_L2;
        Vector3d L1_t_L2;
        L1_R_L2=W_R_L1.inverse()*W_R_L2;
        L1_t_L2<<_t1-t1,_t2-t2,_t3-t3;
        file2<<data[i*13]<<" "<<L1_R_L2(0,0)<<" "<<L1_R_L2(0,1)<<" "<<L1_R_L2(0,2)<<" "<<L1_R_L2(1,0)<<" "<<
        L1_R_L2(1,1)<<" "<<L1_R_L2(1,2)<<" "<<L1_R_L2(2,0)<<" "<<L1_R_L2(2,1)<<" "<<L1_R_L2(2,2)<<" "<<
        L1_t_L2[0]<<" "<<L1_t_L2[1]<<" "<<L1_t_L2[2]<<endl;
    }
    return 0;
}
