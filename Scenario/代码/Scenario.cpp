#include "Scenario.h"
#include<QFile>
#include<QString>
#include<QDomDocument>
#include<QMessageBox>
#include <QDir>
#include <QDebug>
#include"osg/CTarget.h"
#include "ModelManager.h"
#if _MSC_VER
#pragma execution_character_set("utf-8")
#endif

Scenario::Scenario(osgEarth::MapNode* mapNode,ModelManager* modelManager)
{
   _mapNode = mapNode;
   _modelManager = modelManager;

   _group = new osg::Group;
   _group->setDataVariance(osg::Object::DYNAMIC);
}

/////////////////////////////////////////////////////
/// \brief Scenario::loadScenarioFile
/// \param fileName
/// \return
/// 从文件中加载想定
bool Scenario::loadScenarioFile(const QString &fileName)
{


    QFile xmlFile(fileName);

    if(!xmlFile.open(QFile::ReadOnly | QFile::Text))
    {
        return false;
    }
    //获取想定的文件夹路径
    int index = fileName.lastIndexOf('.');
    QString sceFolderPath = fileName.left(index);


    QString error;
    int row=0,column=0;
    QDomDocument doc;
    if(!doc.setContent(&xmlFile,false,&error,&row,&column))
    {
        QMessageBox::information(NULL,QObject::tr("读取想定文件错误"),
                                 QObject::tr("parse the file \"%1\" failed at line %2  and column %3").
                                 arg(fileName).arg(row).arg(column));
        xmlFile.close();
        return false;
    }
    //读取实体列表

    QDomElement root = doc.documentElement();
    if(root.isNull())
    {
        xmlFile.close();
        return false;
    }
    QDomElement EntityListModel = root.firstChildElement("EntityList");
    QDomElement  temp;
    if(!EntityListModel.isNull())
    {
        temp= EntityListModel.firstChildElement("Entity");
        while(!temp.isNull())
        {
            QString str = temp.text();
            QString entityFilePath = sceFolderPath;
            entityFilePath.append("/").append(str);
            ParseEntityFile(entityFilePath);
            temp = temp.nextSiblingElement("Entity");

        }

    }
    xmlFile.close();
    qDebug()<<"读取想定文件成功";

    return false;

}


///////////////////////
/// \brief Scenario::ParseEntityFile
/// \param fileName 要加载的实体文件名
/// \return
///解析实体定义文件
bool Scenario::ParseEntityFile(const QString &fileName)
{
    QFile xmlFile(fileName);

    if(!xmlFile.open(QFile::ReadOnly | QFile::Text))
    {
        return false;
    }

    QString error;
    int row=0,column=0;
    QDomDocument doc;
    if(!doc.setContent(&xmlFile,false,&error,&row,&column))
    {
        QMessageBox::information(NULL,QObject::tr("读取实体文件错误"),
                                 QObject::tr("parse the file \"%1\" failed at line %2  and column %3").
                                 arg(fileName).arg(row).arg(column));
        xmlFile.close();
        return false;
    }
    QDomElement root = doc.documentElement();
    if(root.isNull())
    {
        xmlFile.close();
        return false;
    }
    QDomElement temp = root.firstChildElement("Name");
    QString entityName = temp.isNull()?tr("未命名"):temp.text();

    int part = 0; //默认是红方 0是红方 1是蓝防 2是中立方
    temp = root.firstChildElement("Part");
    if(!temp.isNull())
    {
        if(temp.text().toUpper()==QString("RED"))
        {
             part = 0;
        }else if(temp.text().toUpper()==QString("BLUE"))
        {
            part = 1;
        }else
        {
            part = 2;
        }
    }
    temp = root.firstChildElement("ModelName");
    QString modelName = temp.isNull()?tr("UAV"):temp.text();

    QDomElement initEle = root.firstChildElement("Init");
    if(!initEle.isNull())
    {
        double lon = 0.0,lat = 0.0,alt = 0.0;
        QDomElement posEle = initEle.firstChildElement("Pos");
        if(!posEle.isNull())
        {
            temp = posEle.firstChildElement("Lon");
            lon = temp.isNull()?0.0:temp.text().toDouble();

            temp = posEle.firstChildElement("Lat");
            lat = temp.isNull()?0.0:temp.text().toDouble();

            temp = posEle.firstChildElement("Alt");
            alt = temp.isNull()?0.0:temp.text().toDouble();
        }
        temp = initEle.firstChildElement("Speed");
        double speed = temp.isNull()?0.0:temp.text().toDouble();

        temp = initEle.firstChildElement("Head");
        double heading =  temp.isNull()?0.0:temp.text().toDouble();

        osg::Vec4 color ;
        if(part==0)color= osg::Vec4(1.0,0.0,0.0,0.4);
        else if(part==1)color= osg::Vec4(0.0,0.0,1.0,0.4);
        else color= osg::Vec4(1.0,1.0,1.0,0.4);
        osg::Node* model =NULL;
        if(_modelManager)
        {
           model = _modelManager->GetModelByName(modelName.toStdString());
        }
        int id = _entityList.size()+10000;
        if(model!=NULL && _mapNode != NULL )
        {
           Entity* ent = new CTarget(_mapNode,color,id,
                                     model,entityName);
             ent->Init();
           _group->addChild(ent->GetEntityGroup());
           ent->SetPosition(lon,lat,alt);
           ent->SetOrientation(heading,0,0);
           ent->Update();
           ent->Show(true);
           _entityList.push_back(ent);
        }

    }
    xmlFile.close();
    return true;
}

bool Scenario::loadAreaFile(const QString &fileName)
{
    //读取XML

//       QFile xmlFile(fileName);
//       if(!xmlFile.open(QFile::ReadOnly | QFile::Text))
//       {
//           return;
//       }
//       QString error;int row = 0,column = 0;
//       QDomDocument doc;
//       if(!doc.setContent(&xmlFile,false,&error,&row,&column))
//       {
//           QMessageBox::information(NULL,QObject::tr("读取空域文件%1错误").arg(XfileName),
//                                    QObject::tr("在%1行%2列发生解析错误").
//                                    arg(row).arg(column));
//           xmlFile.close();
//           return;
//       }
//       QDomElement rootElement = doc.documentElement();
//       if(!rootElement.isNull())
//       {
//           //获取空域名称
//           QDomElement nameElement =
//                   rootElement.firstChildElement(QObject::tr("名称"));
//           if(!nameElement.isNull())
//           {
//               QByteArray name = nameElement.text().toUtf8();
//               _spaceArea->_name.assign(name.data());
//           }else {
//               QDomElement nameElement =
//                       rootElement.firstChildElement(QObject::tr("MC"));
//               if(!nameElement.isNull())
//               {
//                   QByteArray name = nameElement.text().toUtf8();
//                   _spaceArea->_name.assign(name.data());
//               }
//           }

//           //获取航线的RGBA
//           QDomElement colorElement =
//                   rootElement.firstChildElement("COLOR");
//           if(!colorElement.isNull())
//           {
//               QDomElement temp = colorElement.firstChildElement("R");
//               double r = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("G");
//               double g = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("B");
//               double b = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("A");
//               double a = temp.isNull() ? 1.0 : temp.text().toFloat();

//               _spaceArea->setWallColor(osg::Vec4(r,g,b,a));


//               temp = colorElement.firstChildElement("LR");
//               double lR = temp.isNull() ? r*0.7 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("LG");
//               double lG = temp.isNull() ? g*0.7 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("LB");
//               double lB = temp.isNull() ? b*0.7 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("LA");
//               double lA = temp.isNull() ? 1.0 : temp.text().toFloat();

//               _spaceArea->setLineColor(osg::Vec4(lR,lG,lB,lA));

//               temp = colorElement.firstChildElement("PR");
//               double pR = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("PG");
//               double pG = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("PB");
//               double pB = temp.isNull() ? 1.0 : temp.text().toFloat();

//               temp = colorElement.firstChildElement("PA");
//               double pA = temp.isNull() ? 0.0 : temp.text().toFloat();

//           }
//           //判断是圆形还是多边形
//           QDomElement centerElement =
//                   rootElement.firstChildElement("CenterPT");
//           double centerL = -181.0,centerB=-91.0,range = 0.0;
//           if(!centerElement.isNull())//圆形
//           {
//               QDomElement temp = centerElement.firstChildElement("L");
//               //centerL = temp.isNull() ? -181.0 : DMS2Double(const_cast<char*>(temp.text().toStdString().c_str()));
//               temp = centerElement.firstChildElement("B");
//              // centerB = temp.isNull() ? -181.0 : DMS2Double(const_cast<char*>(temp.text().toStdString().c_str()));
//               if(centerL>-180.0 && centerL <180.0 &&
//                       centerB <90.0 && centerB>-90.0)
//               {
//                   temp = rootElement.firstChildElement("Range");
//                   range = (temp.isNull()?0:temp.text().toDouble());

//                   //计算好圆形空域的顶点
//                   for (int i=0;i<360;i++)
//                   {
//                       _spaceArea->_originPointList->push_back(osg::Vec2(centerL + range/108000.0*cos(osg::DegreesToRadians(i*10.0)),
//                                                                         centerB + range/108000.0*sin(osg::DegreesToRadians(i*10.0)))
//                                                               );
//                   }
//               }
//           }
//           else
//           {
//               QDomElement pointEle = rootElement.firstChildElement("PT");
//               QDomElement temp;
//               double L,B;
//               while(!pointEle.isNull())
//               {
//                   temp = pointEle.firstChildElement("L");
//                  // L = temp.isNull() ? 0.0 : DMS2Double(const_cast<char*>(temp.text().
//                  //                                                        toStdString().c_str()));

//                  // temp = pointEle.firstChildElement("B");
//                  // B = temp.isNull() ? 0.0 : DMS2Double(const_cast<char*>(temp.text().
//                  //                                                        toStdString().c_str()));

//                   _spaceArea->_originPointList->push_back(osg::Vec2(L,B));

//                   pointEle = pointEle.nextSiblingElement("PT");
//               }
//           }

//           //目前只做单层空域的解析
//           QDomElement layerElement = rootElement.firstChildElement("Layer");
//           if(!layerElement.isNull())
//           {
//               QDomElement temp =  layerElement.firstChildElement("Low");
//               double low = temp.isNull() ? 0.0 :temp.text().toDouble();
//               temp =  layerElement.firstChildElement("High");
//               double high = temp.isNull() ? 0.0 :temp.text().toDouble();

//               if(low < 0.0)
//                   low = 0.0;
//               if(high<=low)
//                   high = low+10000.0;

//               _spaceArea->_lowHeight = low;
//               _spaceArea->_highHeight = high;
//           }
//       }
//       xmlFile.close();
    return true;
}

void Scenario::Clear()
{

}

