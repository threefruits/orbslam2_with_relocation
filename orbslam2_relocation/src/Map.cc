/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include <mutex>
#include "Converter.h"
namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}
void Map::Save(const string& filename)
{
    //输出地图保存的信息
    cerr << "Map Saving to " << filename << endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out | ios::binary);
    //输出地图点的数目
    cerr << "The number of MapPoints is :" << mspMapPoints.size() << endl;

    //地图点的数目
    unsigned long int nMapPoints = mspMapPoints.size();
    f.write((char*)&nMapPoints, sizeof(nMapPoints));
    //依次保存MapPoints
    for (auto mp : mspMapPoints)
        //调用SaveMapPoint函数
        SaveMapPoint(f, mp);

    //获取每一个MapPoints的索引值，即从0开始计数，初始化了mmpnMapPointsIdx
    GetMapPointsIdx();

    //输出关键帧的数量
    cerr << "The number of KeyFrames:" << mspKeyFrames.size() << endl;
    //关键帧的数目
    unsigned long int nKeyFrames = mspKeyFrames.size();
    f.write((char*)&nKeyFrames, sizeof(nKeyFrames));

    //依次保存关键帧KeyFrames
    for (auto kf : mspKeyFrames)
        SaveKeyFrame(f, kf);

    for (auto kf : mspKeyFrames)
    {
        //获得当前关键帧的父节点，并保存父节点的ID
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if (parent)
            parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));
        //获得当前关键帧的关联关键帧的大小，并依次保存每一个关联关键帧的ID和weight；
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con, sizeof(nb_con));
        for (auto ckf : kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    f.close();
    cerr << "Map Saving Finished!" << endl;
}
void Map::SaveMapPoint(ofstream& f, MapPoint* mp)
{
    //保存当前MapPoint的ID和世界坐标值
    f.write((char*)&mp->mnId, sizeof(mp->mnId));
    cv::Mat mpWorldPos = mp->GetWorldPos();
    f.write((char*)&mpWorldPos.at<float>(0), sizeof(float));
    f.write((char*)&mpWorldPos.at<float>(1), sizeof(float));
    f.write((char*)&mpWorldPos.at<float>(2), sizeof(float));
}

void Map::GetMapPointsIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    unsigned long int i = 0;
    for (auto mp : mspMapPoints)
    {
        mmpnMapPointsIdx[mp] = i;
        i += 1;
    }
}

void Map::SaveKeyFrame(ofstream& f, KeyFrame* kf)
{
    //保存当前关键帧的ID和时间戳
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
    //保存当前关键帧的位姿矩阵
    cv::Mat Tcw = kf->GetPose();
    //通过四元数保存旋转矩阵
    std::vector<float> Quat = Converter::toQuaternion(Tcw);
    for (int i = 0; i < 4; i++)
        f.write((char*)&Quat[i], sizeof(float));
    //保存平移矩阵
    for (int i = 0; i < 3; i++)
        f.write((char*)&Tcw.at<float>(i, 3), sizeof(float));

    //直接保存旋转矩阵
    //  for ( int i = 0; i < Tcw.rows; i ++ )
    //  {
    //      for ( int j = 0; j < Tcw.cols; j ++ )
    //      {
    //              f.write((char*)&Tcw.at<float>(i,j), sizeof(float));
    //              //cerr<<"Tcw.at<float>("<<i<<","<<j<<"):"<<Tcw.at<float>(i,j)<<endl;
    //      }
    //    }

    //保存当前关键帧包含的ORB特征数目
    // cerr<<"kf->N:"<<kf->N<<endl;
    f.write((char*)&kf->N, sizeof(kf->N));
    //保存每一个ORB特征点
    for (int i = 0; i < kf->N; i++)
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle, sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        //保存当前特征点的描述符
        for (int j = 0; j < kf->mDescriptors.cols; j++)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i, j),
                    sizeof(char));

        //保存当前ORB特征对应的MapPoints的索引值
        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL)
            mnIdx = ULONG_MAX;
        else
            mnIdx = mmpnMapPointsIdx[mp];

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }
}
void Map::Load ( const string &filename, SystemSetting* mySystemSetting )
 {
     cerr << "Map reading from:"<<filename<<endl;
     ifstream f;
     f.open( filename.c_str());

    //  if (!ifs.good()) 
    //  {
    //      exit(-2);
    //  }

     //按照保存的顺序，先读取MapPoints的数目；
     unsigned long int nMapPoints;
     f.read((char*)&nMapPoints, sizeof(nMapPoints));
 
     //依次读取每一个MapPoints，并将其加入到地图中
     cerr<<"The number of MapPoints:"<<nMapPoints<<endl;
     for ( unsigned int i = 0; i < nMapPoints; i ++ )
     {
         MapPoint* mp = LoadMapPoint(f);
         AddMapPoint(mp);
     }
 
     //获取所有的MapPoints；
     std::vector<MapPoint*> vmp = GetAllMapPoints();
 
     //读取关键帧的数目；
     unsigned long int nKeyFrames;
     f.read((char*)&nKeyFrames, sizeof(nKeyFrames));
     cerr<<"The number of KeyFrames:"<<nKeyFrames<<endl;
 
     //依次读取每一关键帧，并加入到地图；
     vector<KeyFrame*>kf_by_order;
     for( unsigned int i = 0; i < nKeyFrames; i ++ )
     {
         KeyFrame* kf = LoadKeyFrame(f, mySystemSetting);
         AddKeyFrame(kf);
         kf_by_order.push_back(kf);
     }
 
     cerr<<"KeyFrame Load OVER!"<<endl;
     //读取生长树；
     map<unsigned long int, KeyFrame*> kf_by_id;
     for ( auto kf: mspKeyFrames )
         kf_by_id[kf->mnId] = kf;
     cerr<<"Start Load The Parent!"<<endl;
     for( auto kf: kf_by_order )
     {
         //读取当前关键帧的父节点ID；
         unsigned long int parent_id;
         f.read((char*)&parent_id, sizeof(parent_id));
 
         //给当前关键帧添加父节点关键帧；
         if ( parent_id != ULONG_MAX )
             kf->ChangeParent(kf_by_id[parent_id]);
 
         //读取当前关键帧的关联关系；
         //先读取当前关键帧的关联关键帧的数目；
         unsigned long int nb_con;
         f.read((char*)&nb_con, sizeof(nb_con));
         //然后读取每一个关联关键帧的ID和weight，并把该关联关键帧加入关系图中；
         for ( unsigned long int i = 0; i < nb_con; i ++ )
         {
             unsigned long int id;
             int weight;
             f.read((char*)&id, sizeof(id));
             f.read((char*)&weight, sizeof(weight));
             kf->AddConnection(kf_by_id[id],weight);
         }
    }
    cerr<<"Parent Load OVER!"<<endl;
    for ( auto mp: vmp )
    {
        if(mp)
        {
             mp->ComputeDistinctiveDescriptors();
             mp->UpdateNormalAndDepth();
         }
    }
     f.close();
     cerr<<"Load IS OVER!"<<endl;
     return;
 }
 MapPoint* Map::LoadMapPoint( ifstream &f )
 {
         //主要包括MapPoints的位姿和ID；
         cv::Mat Position(3,1,CV_32F);
         long unsigned int id;
         f.read((char*)&id, sizeof(id));
 
         f.read((char*)&Position.at<float>(0), sizeof(float));
         f.read((char*)&Position.at<float>(1), sizeof(float));
         f.read((char*)&Position.at<float>(2), sizeof(float));
 
         //初始化一个MapPoint，并设置其ID和Position；
         MapPoint* mp = new MapPoint(Position, this );
         mp->mnId = id;
         mp->SetWorldPos( Position );
 
         return mp;
 }
KeyFrame* Map::LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting )
 {
     //声明一个初始化关键帧的类initkf；
     InitKeyFrame initkf(*mySystemSetting);
 
     //按照保存次序，依次读取关键帧的ID和时间戳；
     f.read((char*)&initkf.nId, sizeof(initkf.nId));
     f.read((char*)&initkf.TimeStamp, sizeof(double));
 
     //读取关键帧位姿矩阵；
     cv::Mat T = cv::Mat::zeros(4,4,CV_32F);
     std::vector<float> Quat(4);
     //Quat.reserve(4);
     for ( int i = 0; i < 4; i ++ )
         f.read((char*)&Quat[i],sizeof(float));
     cv::Mat R = Converter::toCvMat( Quat );
     for ( int i = 0; i < 3; i ++ )
         f.read((char*)&T.at<float>(i,3),sizeof(float));
     for ( int i = 0; i < 3; i ++ )
         for ( int j = 0; j < 3; j ++ )
             T.at<float>(i,j) = R.at<float>(i,j);
     T.at<float>(3,3) = 1;
 
 //    for ( int i = 0; i < 4; i ++ )
 //    {
 //      for ( int j = 0; j < 4; j ++ )
 //      {
 //              f.read((char*)&T.at<float>(i,j), sizeof(float));
 //              cerr<<"T.at<float>("<<i<<","<<j<<"):"<<T.at<float>(i,j)<<endl;
 //      }
 //    }
 
     //读取当前关键帧特征点的数目；
     f.read((char*)&initkf.N, sizeof(initkf.N));
     initkf.vKps.reserve(initkf.N);
     initkf.Descriptors.create(initkf.N, 32, CV_8UC1);
     vector<float>KeypointDepth;
 
     std::vector<MapPoint*> vpMapPoints;
     vpMapPoints = vector<MapPoint*>(initkf.N,static_cast<MapPoint*>(NULL));
     //依次读取当前关键帧的特征点和描述符；
     std::vector<MapPoint*> vmp = GetAllMapPoints();
     for(int i = 0; i < initkf.N; i ++ )
     {
         cv::KeyPoint kp;
         f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
         f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
         f.read((char*)&kp.size, sizeof(kp.size));
         f.read((char*)&kp.angle,sizeof(kp.angle));
         f.read((char*)&kp.response, sizeof(kp.response));
         f.read((char*)&kp.octave, sizeof(kp.octave));
 
         initkf.vKps.push_back(kp);
 
         //根据需要读取特征点的深度值；
         //float fDepthValue = 0.0;
         //f.read((char*)&fDepthValue, sizeof(float));
         //KeypointDepth.push_back(fDepthValue);
 
         //读取当前特征点的描述符；
         for ( int j = 0; j < 32; j ++ )
                 f.read((char*)&initkf.Descriptors.at<unsigned char>(i,j),sizeof(char));
 
         //读取当前特征点和MapPoints的对应关系；
         unsigned long int mpidx;
         f.read((char*)&mpidx, sizeof(mpidx));
 
         //从vmp这个所有的MapPoints中查找当前关键帧的MapPoint，并插入
         if( mpidx == ULONG_MAX )
                 vpMapPoints[i] = NULL;
         else
                 vpMapPoints[i] = vmp[mpidx];
     }
 
     initkf.vRight = vector<float>(initkf.N,-1);
     initkf.vDepth = vector<float>(initkf.N,-1);
     //initkf.vDepth = KeypointDepth;
     initkf.UndistortKeyPoints();
     initkf.AssignFeaturesToGrid();
 
     //使用initkf初始化一个关键帧，并设置相关参数
     KeyFrame* kf = new KeyFrame( initkf, this, NULL, vpMapPoints );
     kf->mnId = initkf.nId;
     kf->SetPose(T);
     kf->ComputeBoW();

     for ( int i = 0; i < initkf.N; i ++ )
     {
         if ( vpMapPoints[i] )
         {
             vpMapPoints[i]->AddObservation(kf,i);
             if( !vpMapPoints[i]->GetReferenceKeyFrame())
                 vpMapPoints[i]->SetReferenceKeyFrame(kf);
         }
     }
     return kf;
 }
} //namespace ORB_SLAM
