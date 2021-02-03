/*
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ZED_OD_INFO_HPP
#define ZED_OD_INFO_HPP

#include <string>
#include <QObject>

#include "visibility_control.hpp"

#include <zed_interfaces/msg/objects_stamped.hpp>

#include <rviz_rendering/objects/shape.hpp>
//#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>


#include <rviz_rendering/objects/object.hpp>

namespace Ogre {
    class SceneManager;
    class SceneNode;
}

namespace rviz_plugin_zed_od
{
namespace displays
{

typedef std::shared_ptr<rviz_rendering::Shape> shapePtr;
typedef std::shared_ptr<rviz_rendering::BillboardLine> linePtr;

class ZED_OD_PLUGIN_PUBLIC ZedOdInfo : public QObject {
    Q_OBJECT

public:
    explicit ZedOdInfo(zed_interfaces::msg::Object &obj,
                       Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node=NULL);
    virtual ~ZedOdInfo();

    void updateShowLabel(bool show);
    void updateAlpha(float alpha);
    void updateInfo(zed_interfaces::msg::Object& obj);
    void updateShowBBox(bool show);
    void updateShowSkeleton(bool show);
    void updateLinkSize(float newval);
    void updateJointRadius(float newval);
    void updateLabelScale(float newval);

protected:
    void calculateColor();

private:
    std::string mObjName;

    //shapePtr mBBox;
    std::shared_ptr<rviz_rendering::MovableText> mLabel;

    std::vector<shapePtr> mBBoxCorners;
    std::vector<linePtr> mBBoxLines;

    std::vector<shapePtr> mSkelJoints;
    std::vector<linePtr> mSkelLinks;

    float mAlpha = 0.75f;
    Ogre::ColourValue mColorBBox;
    Ogre::ColourValue mColorSkel;

    Ogre::SceneManager* mSceneManager=nullptr;
    Ogre::SceneNode* mParentNode=nullptr;
    Ogre::SceneNode* mSceneNode=nullptr;
    Ogre::SceneNode* mSkelSceneNode=nullptr;
    Ogre::SceneNode* mBBoxSceneNode=nullptr;
    Ogre::SceneNode* mPivotSceneNode=nullptr;

    bool mShowLabel=true;
    bool mShowBBox=true;
    bool mShowSkel=true;

    shapePtr mPivot;

    int16_t mLabelId;

    float mLabelScale = 2.5;
    float mJointRadius = 0.1;
    float mLinkSize = 0.05;

    const float mSkelScale = 0.5f;
    const int mSkelColOffset = 50;

    // Unique identifier for each object
    static uint64_t mObjIdx;
};

enum class BODY_PARTS {
    NOSE = 0,
    NECK = 1,
    RIGHT_SHOULDER = 2,
    RIGHT_ELBOW = 3,
    RIGHT_WRIST = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    RIGHT_HIP = 8,
    RIGHT_KNEE = 9,
    RIGHT_ANKLE = 10,
    LEFT_HIP = 11,
    LEFT_KNEE = 12,
    LEFT_ANKLE = 13,
    RIGHT_EYE = 14,
    LEFT_EYE = 15,
    RIGHT_EAR = 16,
    LEFT_EAR = 17,
    LAST = 18
};

const std::vector<std::pair< BODY_PARTS, BODY_PARTS>> BODY_BONES
{
    {
        BODY_PARTS::NOSE, BODY_PARTS::NECK
    },
    {
        BODY_PARTS::NECK, BODY_PARTS::RIGHT_SHOULDER
    },
    {
        BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::RIGHT_ELBOW
    },
    {
        BODY_PARTS::RIGHT_ELBOW, BODY_PARTS::RIGHT_WRIST
    },
    {
        BODY_PARTS::NECK, BODY_PARTS::LEFT_SHOULDER
    },
    {
        BODY_PARTS::LEFT_SHOULDER, BODY_PARTS::LEFT_ELBOW
    },
    {
        BODY_PARTS::LEFT_ELBOW, BODY_PARTS::LEFT_WRIST
    },
    {
        BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::RIGHT_HIP
    },
    {
        BODY_PARTS::RIGHT_HIP, BODY_PARTS::RIGHT_KNEE
    },
    {
        BODY_PARTS::RIGHT_KNEE, BODY_PARTS::RIGHT_ANKLE
    },
    {
        BODY_PARTS::LEFT_SHOULDER, BODY_PARTS::LEFT_HIP
    },
    {
        BODY_PARTS::LEFT_HIP, BODY_PARTS::LEFT_KNEE
    },
    {
        BODY_PARTS::LEFT_KNEE, BODY_PARTS::LEFT_ANKLE
    },
    {
        BODY_PARTS::RIGHT_SHOULDER, BODY_PARTS::LEFT_SHOULDER
    },
    {
        BODY_PARTS::RIGHT_HIP, BODY_PARTS::LEFT_HIP
    },
    {
        BODY_PARTS::NOSE, BODY_PARTS::RIGHT_EYE
    },
    {
        BODY_PARTS::RIGHT_EYE, BODY_PARTS::RIGHT_EAR
    },
    {
        BODY_PARTS::NOSE, BODY_PARTS::LEFT_EYE
    },
    {
        BODY_PARTS::LEFT_EYE, BODY_PARTS::LEFT_EAR
    }
};

} // namespace displays
} // namespace rviz_plugin_zed_od

#endif // #ifndef ZED_OD_INFO_HPP
