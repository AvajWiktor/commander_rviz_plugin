#include <iostream>
#include "commander_rviz_plugins/commander_display.hpp"

namespace commander_rviz_plugins
{
    using rviz_common::properties::StatusLevel;

    CommanderDisplay::CommanderDisplay()
    {
        near_clip_property_ = new rviz_common::properties::FloatProperty("Near Clip", 0.0, "", this);

        far_clip_property_ = new rviz_common::properties::FloatProperty("Far Clip", 0.0, "", this);
    }

    CommanderDisplay::~CommanderDisplay()
    {

    }
    void CommanderDisplay::onInitialize()
    {
        try
        {
            setupCommanderPanel();
            initializeButtons();
            setupLookingCommanderPanel();

            setStatus(StatusLevel::Ok, "DD", "OK");
        }
        catch (const std::exception &e)
        {
            setStatus(StatusLevel::Error, "DD", e.what());
        }
    }
    void CommanderDisplay::initializeButtons()
    {
        leftButton_ = std::make_unique<QPushButton>("\u25C0");
        rightButton_ = std::make_unique<QPushButton>("\u25B6");
        upButton_ = std::make_unique<QPushButton>("\u25B2");
        downButton_ = std::make_unique<QPushButton>("\u25BC");
        velocitySlider_ = std::make_unique<QSlider>(Qt::Horizontal);
        velocitySlider_->setFocusPolicy(Qt::StrongFocus);
        velocitySlider_->setTickPosition(QSlider::TicksBothSides);
        velocitySlider_->setTickInterval(10);
        velocitySlider_->setSingleStep(1);
        velocitySlider_->setMaximum(100);
        velocitySlider_->setValue(100);
        std::string temp_s = "Velocity Scale: "+std::to_string(velocityScale_);
        velocitySliderLabel_ = std::make_unique<QLabel>();
        velocitySliderLabel_->setText(QString(&temp_s[0]));
        velocitySliderLabel_->setAlignment(Qt::AlignCenter);
        connect(leftButton_.get(), &QPushButton::pressed, this, &CommanderDisplay::leftOnClick);
        connect(rightButton_.get(), &QPushButton::pressed, this, &CommanderDisplay::rightOnClick);
        connect(upButton_.get(), &QPushButton::pressed, this, &CommanderDisplay::upOnClick);
        connect(downButton_.get(), &QPushButton::pressed, this, &CommanderDisplay::downOnClick);

        connect(leftButton_.get(), &QPushButton::released, this, &CommanderDisplay::leftOnRelease);
        connect(rightButton_.get(), &QPushButton::released, this, &CommanderDisplay::rightOnRelease);
        connect(upButton_.get(), &QPushButton::released, this, &CommanderDisplay::upOnRelease);
        connect(downButton_.get(), &QPushButton::released, this, &CommanderDisplay::downOnRelease);

        connect(velocitySlider_.get(), &QDial::valueChanged, this, &CommanderDisplay::onVelocityScaleChange);

        leftButton_ ->setAutoRepeat(true);
        rightButton_->setAutoRepeat(true);
        upButton_   ->setAutoRepeat(true);
        downButton_ ->setAutoRepeat(true);
    }

    void CommanderDisplay::setupCommanderPanel()
    {
        commander_panel_ = std::make_unique<rviz_common::RenderPanel>();
        commander_panel_->initialize(context_, true);
        setAssociatedWidget(commander_panel_.get());

        static int count = 0;
        commander_panel_->getRenderWindow()->setObjectName("CommanderPlayRenderWindow" + QString::number(count++));
        commander_panel_->setViewController(context_->getViewManager()->getCurrent());
    }

    void CommanderDisplay::setupLookingCommanderPanel()
    {
        QWidget *q = new QWidget;
        QGridLayout *mainLayout = new QGridLayout;
        mainLayout->addWidget(leftButton_.get(),1,0,1,2);
        mainLayout->addWidget(rightButton_.get(),1,2,1,2);
        mainLayout->addWidget(upButton_.get(),0,1,1,2);
        mainLayout->addWidget(downButton_.get(),2,1,1,2);
        mainLayout->addWidget(velocitySliderLabel_.get(), 3,0,1,4);
        mainLayout->addWidget(velocitySlider_.get(), 4,0,1,4);
        q->setLayout(mainLayout);
        //commander_panel_->setLayout(mainLayout);
        looking_commander_panel_ = getAssociatedWidgetPanel();
        looking_commander_panel_->setContentWidget(q);  //(new QWidget(looking_commander_panel_));
        looking_commander_panel_->setFloating(true);
        looking_commander_panel_->setGeometry(win_x_, win_y_, win_w_, win_h_);
        looking_commander_panel_->setWindowState(looking_commander_panel_->windowState() | Qt::WindowNoState);
    }
} // namespace commander_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(commander_rviz_plugins::CommanderDisplay, rviz_common::Display)
