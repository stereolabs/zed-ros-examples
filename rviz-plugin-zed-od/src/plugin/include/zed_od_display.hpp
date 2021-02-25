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

#ifndef ZED_OD_PLUGIN_HPP
#define ZED_OD_PLUGIN_HPP

#include <zed_interfaces/ObjectsStamped.h>

#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <map>

#include "zed_od_info.hpp"

namespace rviz_plugin_zed_od
{
namespace displays
{

typedef std::shared_ptr<ZedOdInfo> objectPtr;

class ZedOdDisplay : public
        rviz::MessageFilterDisplay<zed_interfaces::ObjectsStamped> {
    Q_OBJECT

public:
    ZedOdDisplay();
    ~ZedOdDisplay();

    void onInitialize() override;
    void reset() override;

private:
    void processMessage(const zed_interfaces::ObjectsStamped::ConstPtr& msg);
    void createOrUpdateObject(zed_interfaces::Object& obj);
    void invalidateObjs();
    void removeNotValidObjs();

private slots:
    void updateShowSkeleton();
    void updateShowLabel();
    void updateAlpha();
    void updateShowBBox();
    void updateLinkSize();
    void updateJointRadius();
    void updateLabelScale();

protected:
    /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
    void onEnable() override;
    void onDisable() override;

private:
    rviz::FloatProperty* mPropAlpha;
    rviz::BoolProperty* mPropShowSkeleton;
    rviz::BoolProperty* mPropShowLabel;
    rviz::BoolProperty* mPropShowBBox;
    rviz::FloatProperty* mPropLinkSize;
    rviz::FloatProperty* mPropJointRadius;
    rviz::FloatProperty* mPropLabelScale;

    std::map<int16_t,objectPtr> mObjects;
    std::map<int16_t,bool> mObjUpdated;
};

} // namespace displays
} // namespace rviz_plugin_zed_od

#endif // #define ZED_OD_PLUGIN_HPP
