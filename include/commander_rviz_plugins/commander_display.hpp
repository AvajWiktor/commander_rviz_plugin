// Copyright 2022 Borong Yuan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifndef Q_MOC_RUN

#include <OgreCamera.h>
#include <OgreColourValue.h>

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_rendering/logging.hpp"
#include "rviz_rendering/render_window.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <QPushButton>
#include <QSlider>
#include <QtWidgets>
#include <string>
#include <QGridLayout>

#endif

namespace commander_rviz_plugins
{
    class CommanderDisplay : public rviz_common::Display
    {
        Q_OBJECT

    public:
        CommanderDisplay();
        ~CommanderDisplay();


    private Q_SLOTS:
        void leftOnClick(){data_->angular.z = 1.0*velocityScale_;publish_data();};
        void rightOnClick(){data_->angular.z = -1.0*velocityScale_;publish_data();};
        void upOnClick(){data_->linear.x = 1.0*velocityScale_;publish_data();};
        void downOnClick(){data_->linear.x = -1.0*velocityScale_;publish_data();};
        void leftOnRelease(){data_->angular.z = 0.0;publish_data();};
        void rightOnRelease(){data_->angular.z = 0.0;publish_data();};
        void upOnRelease(){data_->linear.x = 0.0;publish_data();};
        void downOnRelease(){data_->linear.x = 0.0;publish_data();};
        void onVelocityScaleChange(int value)
        {
            velocityScale_=(float(value)/100.0);
            std::string temp_s = "Velocity Scale: "+std::to_string(velocityScale_);
            velocitySliderLabel_->setText(QString(&temp_s[0]));
        };
    protected:
        void onInitialize() override;

    private:    
        int win_x_{300}, win_y_{100}, win_w_{300}, win_h_{100}, tile_x_{0}, tile_y_{1};
        void publish_data(){publisher_->publish(*data_);}
        std::unique_ptr<rclcpp::Node> mainNode_ = std::make_unique<rclcpp::Node>("commander_gui_node");

       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ = mainNode_->create_publisher<geometry_msgs::msg::Twist>("joy_teleop/cmd_vel", 10);
        std::unique_ptr<QPushButton> leftButton_;
        std::unique_ptr<QPushButton> rightButton_;
        std::unique_ptr<QPushButton> upButton_;
        std::unique_ptr<QPushButton> downButton_;
        std::unique_ptr<QSlider> velocitySlider_;
        std::unique_ptr<QLabel> velocitySliderLabel_;
        rviz_common::properties::FloatProperty *near_clip_property_;
        rviz_common::properties::FloatProperty *far_clip_property_;

        std::unique_ptr<rviz_common::RenderPanel> commander_panel_;
        rviz_common::PanelDockWidget *looking_commander_panel_;
        float velocityScale_ = 1.0;
        std::unique_ptr<geometry_msgs::msg::Twist> data_ = std::make_unique<geometry_msgs::msg::Twist>();

        void initializeLookingGlass();
        void initializeButtons();
        void setupCommanderPanel();
        void setupLookingCommanderPanel();
    };
} // namespace commander_rviz_plugins
