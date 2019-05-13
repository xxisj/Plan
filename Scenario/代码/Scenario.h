#ifndef SCENARIO_H
#define SCENARIO_H

#include <QObject>
#include "OSG/OSG_Include.h"
#include "osg/Entity.h"
class ModelManager;

class Scenario : public QObject
{
    Q_OBJECT
public:
    explicit Scenario(osgEarth::MapNode* mapNode,ModelManager* modelManager);
    bool loadScenarioFile(const QString&  fileName);/*加载想定*/
    osg::Node* GetScenarioScene(){return _group.valid()?_group.get():NULL;}
    void Clear();
signals:

public slots:
private:
   bool ParseEntityFile(const QString&  fileName);
   bool loadAreaFile(const QString& fileName);
private:
   vector<Entity*> _entityList;
   osgEarth::MapNode* _mapNode;
   ModelManager* _modelManager;
   osg::ref_ptr<osg::Group> _group;


};

#endif // SCENARIO_H
