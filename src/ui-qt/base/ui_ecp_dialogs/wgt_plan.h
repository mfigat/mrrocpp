#ifndef WGT_PLAN_H
#define WGT_PLAN_H

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>

#include "../wgt_base.h"
#include "../interface.h"

#include "base/lib/exception.h"

#include "ui_wgt_plan.h"

#include "application/swarmitfix_plan/plan.hxx"

class wgt_plan : public wgt_base
{
Q_OBJECT

public:
	wgt_plan(mrrocpp::ui::common::Interface& _interface, QWidget *parent = 0);
	~wgt_plan();

	void hideEvent(QHideEvent * event);
	void my_open(bool set_on_top = false);

	REGISTER_FATAL_ERROR(plan_item_out_of_range, "plan item value is out of range");

private:
	//! Clipboard cache for copy-paste of plan items.
	struct {
		struct common {
			bool present;
			std::string comment;
			unsigned int agent;
		};
		//! PKM+HEAD plan item.
		struct : common {
			float x,y,z,alpha,beta,gamma,head;
		} pkm_item;

		//! MBASE+BENCH plan item.
		struct : common {
			double pkmTheta;
			unsigned int rows[3], columns[3];
		} mbase_item;
	} clipboard;

	Ui::wgt_planClass* ui;

	//! Deactivate widget into idle mode
	void reply();

	//! Reload inputs with original request
	void reload();

	//! Check if value is within widget limits.
	template<class WIDGET_T, class VALUE_T>
	void checkInputWidgetLimits(const WIDGET_T & widget, const VALUE_T value)
	{
		if ((value < widget.minimum()) ||(value > widget.maximum())) {
			std::stringstream msg;

			msg << "Input widget value " << value
					<< " out of range <" << widget.minimum()
					<< ".."
					<< widget.maximum()
					<< ">";

			std::cerr << msg.str() << std::endl;

			interface.ui_msg->message(lib::NON_FATAL_ERROR, msg.str());
			BOOST_THROW_EXCEPTION(plan_item_out_of_range());
		}
	}

	//! Disallow execution when setting input widgets by code.
	bool finetuning_active;

private slots:

	// Navigation buttons.
	void on_pushButton_prev_clicked();
	void on_pushButton_next_clicked();
	void on_pushButton_exec_clicked();
	void on_pushButton_save_clicked();
	void on_pushButton_reload_clicked();
	void on_pin_input_valueChanged(int i);

	// PKM fine-tuning.
	void on_x_input_valueChanged(double value);
	void on_y_input_valueChanged(double value);
	void on_z_input_valueChanged(double value);
	void on_alpha_input_valueChanged(double value);
	void on_beta_input_valueChanged(double value);
	void on_gamma_input_valueChanged(double value);
	void on_head_input_valueChanged(double value);

	// MBASE fine-tuning.
	void on_pkmTheta_input_valueChanged(double value);

	// Support widgets.
	void on_pushbutton_clearComment_clicked();
	void on_pushbutton_copy_clicked();
	void on_pushbutton_paste_clicked();

	void finetune_pkm();
	void finetune_head();
	void finetune_mbase();
};

#endif // WGT_PLAN_H
