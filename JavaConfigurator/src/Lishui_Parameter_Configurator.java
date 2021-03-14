/*
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

/**
 *
 * @author stancecoke
 */


import java.io.*;
import java.awt.Desktop;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.Date;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;
import javax.swing.text.JTextComponent;
import javax.swing.JTextField;
import javax.swing.JButton;
import javax.swing.ListSelectionModel;
import javax.swing.JLabel;
import javax.swing.ImageIcon;
import javax.imageio.ImageIO;
import javax.swing.AbstractAction;
import java.awt.event.ActionEvent;
import javax.swing.Action;
import java.awt.event.ActionListener;
import java.awt.GridLayout;
import java.awt.Font;
import java.awt.Color;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import javax.swing.SwingConstants;
import javax.swing.JOptionPane;
import javax.swing.JRadioButton;
import javax.swing.ButtonGroup;
import javax.swing.JList;
import javax.swing.JScrollPane;
import java.awt.Dimension;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.List;
import java.util.ArrayList;
import java.io.File;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.DefaultListModel;
import javax.swing.event.ListDataListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.nio.file.Paths;
import java.util.Arrays;
import javax.swing.JCheckBox;
import javax.swing.ListModel;

public class Lishui_Parameter_Configurator extends javax.swing.JFrame {

    /**
     * Creates new form TSDZ2_Configurator
     */
    
    private File experimentalSettingsDir;
    private File lastSettingsFile = null;
    
    DefaultListModel provenSettingsFilesModel = new DefaultListModel();
    DefaultListModel experimentalSettingsFilesModel = new DefaultListModel();
    JList experimentalSettingsList = new JList(experimentalSettingsFilesModel);
    
    	public class FileContainer {

		public FileContainer(File file) {
			this.file = file;
		}
		public File file;

		@Override
		public String toString() {
			return file.getName();
		}
	}

    
    
 
public void loadSettings(File f) throws IOException {
    
     		BufferedReader in = new BufferedReader(new FileReader(f));
		
                TF_TRIGGER_OFFSET.setText(in.readLine());
                TF_TRIGGER_DEFAULT.setText(in.readLine());
                TF_TIMER_PERIOD.setText(in.readLine());
                TF_CAL_BAT_V.setText(in.readLine()); 
                TF_CAL_V.setText(in.readLine());
                TF_CAL_I.setText(in.readLine());
                TF_INDUCTANCE.setText(in.readLine());
                TF_RESISTANCE.setText(in.readLine());
                TF_FLUX_LINKAGE.setText(in.readLine());
                TF_GAMMA.setText(in.readLine());     
                TF_BATTERY_LEVEL_1.setText(in.readLine());
                TF_BATTERY_LEVEL_2.setText(in.readLine()); 
                TF_BATTERY_LEVEL_3.setText(in.readLine()); 
                TF_BATTERY_LEVEL_4.setText(in.readLine()); 
                TF_BATTERY_LEVEL_5.setText(in.readLine());
                TF_P_FACTOR_I_Q.setText(in.readLine());
                TF_I_FACTOR_I_Q.setText(in.readLine());
                TF_P_FACTOR_I_D.setText(in.readLine());
                TF_I_FACTOR_I_D.setText(in.readLine());
                TF_P_FACTOR_PLL.setText(in.readLine());
                TF_I_FACTOR_PLL.setText(in.readLine());   
                TF_P_FACTOR_SPEED.setText(in.readLine());
                TF_I_FACTOR_SPEED.setText(in.readLine());
                TF_SPDSHFT.setText(in.readLine());
                TF_SPEEDFILTER.setText(in.readLine());
                TF_SIXSTEPTHRESHOLD.setText(in.readLine());
                TF_TS_COEF.setText(in.readLine());
                TF_PAS_TIMEOUT.setText(in.readLine());
                TF_PAS_RAMP_END.setText(in.readLine());
                TF_FRAC_HIGH.setText(in.readLine());
                TF_FRAC_LOW.setText(in.readLine());
                TF_THROTTLE_OFFSET.setText(in.readLine());
                TF_THROTTLE_MAX.setText(in.readLine());
                TF_PUSH_ASSIST.setText(in.readLine());
                TF_WHEEL_CIRC.setText(in.readLine());
                TF_GEAR_RATIO.setText(in.readLine());
                TF_SPEED_LIMIT.setText(in.readLine());
                TF_PULSES_PER_REVOLUTION.setText(in.readLine());
                TF_PH_CURRENT_MAX.setText(in.readLine());
                TF_BATTERY_CURRENT_MAX.setText(in.readLine());
                TF_REGEN_CURRENT.setText(in.readLine());
                TF_REGEN_CURRENT_MAX.setText(in.readLine());
                TF_Voltage_MIN.setText(in.readLine());
                TF_Voltage_MAX.setText(in.readLine());
                TF_SPEC_ANGLE.setText(in.readLine());
                RB_DIRDET.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_TORQUESENSOR.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_TQ_ON_AD1.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_JLCD.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_KM5S.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_EBICS.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_KUNTENG.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_BAFANG.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_DEBUG.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_DISABLE_DYN_ADC.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_FAST_LOOP_LOG.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_SPEED_PLL.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_SPEEDTHROTTLE.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_LINEARREGEN.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_SS_EXTERNAL.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_AUTODETECT.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_REVERSE.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_LIMIT_THR.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_THR_OVERRIDE.setSelected(Boolean.parseBoolean(in.readLine()));
                RB_IND_PROFILE.setSelected(Boolean.parseBoolean(in.readLine()));
                TF_PATH_ECLIPSE.setText(in.readLine());
                TF_PATH_STM32_UTILITY.setText(in.readLine());
		in.close();
	}   

public void AddListItem(File newFile) {
        
        experimentalSettingsFilesModel.add(0, new FileContainer(newFile));
    
       // ListModel<String> Liste = expSet.getModel();
        expSet.repaint();
        JOptionPane.showMessageDialog(null,experimentalSettingsFilesModel.toString(),"Titel", JOptionPane.PLAIN_MESSAGE);
}
    
    public Lishui_Parameter_Configurator() {
        initComponents();
        BufferedImage image;
        try {
            image = ImageIO.read (new File ("src/ImageFiles/stancecoke.png"));
            ImageIcon icon = new ImageIcon(image);
            jLabel43.setIcon(icon);
        } catch (IOException ex) {
            Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(Level.SEVERE, null, ex);
        }

        // update lists
        
                experimentalSettingsDir = new File(Paths.get(".").toAbsolutePath().normalize().toString());
		while (!Arrays.asList(experimentalSettingsDir.list()).contains("experimental settings")) {
			experimentalSettingsDir = experimentalSettingsDir.getParentFile();
		}
		File provenSettingsDir = new File(experimentalSettingsDir.getAbsolutePath() + File.separator + "proven settings");
		experimentalSettingsDir = new File(experimentalSettingsDir.getAbsolutePath() + File.separator + "experimental settings");



		for (File file : provenSettingsDir.listFiles()) {
			provenSettingsFilesModel.addElement(new Lishui_Parameter_Configurator.FileContainer(file));

			if (lastSettingsFile == null) {
				lastSettingsFile = file;
			} else {
				if(file.lastModified()>lastSettingsFile.lastModified()){
					lastSettingsFile = file;
				}
			}
		}
 		

                for (File file : experimentalSettingsDir.listFiles()) {
            experimentalSettingsFilesModel.addElement(new Lishui_Parameter_Configurator.FileContainer(file));
	}
        	experimentalSettingsList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		experimentalSettingsList.setLayoutOrientation(JList.VERTICAL);
		experimentalSettingsList.setVisibleRowCount(-1); 
                
                expSet.setModel(experimentalSettingsFilesModel);
        
		JList provenSettingsList = new JList(provenSettingsFilesModel);
		provenSettingsList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		provenSettingsList.setLayoutOrientation(JList.VERTICAL);
		provenSettingsList.setVisibleRowCount(-1);
        
        provSet.setModel(provenSettingsFilesModel);
        jScrollPane2.setViewportView(provSet);
        

        expSet.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
                            	try {
                                int selectedIndex = expSet.getSelectedIndex();
                                experimentalSettingsList.setSelectedIndex(selectedIndex);
					loadSettings(((FileContainer) experimentalSettingsList.getSelectedValue()).file);
					experimentalSettingsList.clearSelection();
				} catch (IOException ex) {
					Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(Level.SEVERE, null, ex);
				}
				experimentalSettingsList.clearSelection();
                                
				//updateDependiencies(false);
			}
		});
        
         provSet.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
                            	try {
                                int selectedIndex = provSet.getSelectedIndex();
                                provenSettingsList.setSelectedIndex(selectedIndex);
					loadSettings(((FileContainer) provenSettingsList.getSelectedValue()).file);
					provenSettingsList.clearSelection();
				} catch (IOException ex) {
					Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(Level.SEVERE, null, ex);
				}
				provenSettingsList.clearSelection();
				//updateDependiencies(false);
			}
		});
         
         
         jButton2.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				{

					int n = JOptionPane.showConfirmDialog(
							null,
							"If you run this function with a brand new controller, the original firmware will be erased. This can't be undone. Are you sure?",
							"",
							JOptionPane.YES_NO_OPTION);

					if (n == JOptionPane.YES_OPTION) {
						try {
							Process process = Runtime.getRuntime().exec("cmd /c start WriteOptionBytes " + TF_PATH_STM32_UTILITY.getText());
						} catch (IOException e1) {
							TF_TS_COEF.setText("Error");
							e1.printStackTrace();
						}
					} else {
						JOptionPane.showMessageDialog(null, "Goodbye");
					}

					// Saving code here
				}
			}
		});
         
                 
         
          jButton1.addActionListener(new ActionListener() {
          
          public void actionPerformed(ActionEvent arg0) {
          				PrintWriter iWriter = null;
                                PrintWriter pWriter = null;
				try {
					//FileWriter fw = new FileWriter("settings.ini");
					//BufferedWriter bw = new BufferedWriter(fw);
                                        
                                        
					File newFile = new File(experimentalSettingsDir + File.separator + new SimpleDateFormat("yyyyMMdd-HHmmssz").format(new Date()) + ".ini");
					//TSDZ2_Configurator ConfiguratorObject = new TSDZ2_Configurator();
                                        //ConfiguratorObject.AddListItem(newFile);
                                        experimentalSettingsFilesModel.add(0, new FileContainer(newFile)); //hier wird nur die neue Datei in die Liste geschrieben...

					iWriter = new PrintWriter(new BufferedWriter(new FileWriter(newFile)));
					pWriter = new PrintWriter(new BufferedWriter(new FileWriter("inc/config.h")));
					pWriter.println("/*\r\n"
							+ " * config.h\r\n"
							+ " *\r\n"
							+ " *  Automatically created by Lishui Parameter Configurator\r\n"
							+ " *  Author: stancecoke\r\n"
							+ " */\r\n"
							+ "\r\n"
							+ "#ifndef CONFIG_H_\r\n"
							+ "#define CONFIG_H_\r\n"
                                                        + "#include \"stdint.h\"\r\n"
                                                        + "#define DISPLAY_TYPE_EBiCS (1<<5)  \r\n"
                                                        + "#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)\r\n"
                                                        + "#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)\r\n"
                                                        + "#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)\r\n"
                                                        + "#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker\r\n"
                                                        + "#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode\r\n"
                                                        + "#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);\r\n"
                                                        + "#define EXTERNAL 1\r\n"
                                                        + "#define INTERNAL 0\r\n"
                                        
                                        
                                        );
                                        
                                        
                                        String text_to_save = "#define TRIGGER_OFFSET_ADC " + TF_TRIGGER_OFFSET.getText();
				        iWriter.println(TF_TRIGGER_OFFSET.getText());
					pWriter.println(text_to_save); 
                                        
                                        text_to_save = "#define TRIGGER_DEFAULT " + TF_TRIGGER_DEFAULT.getText();
				        iWriter.println(TF_TRIGGER_DEFAULT.getText());
					pWriter.println(text_to_save); 
                                             
                                        text_to_save = "#define _T " + TF_TIMER_PERIOD.getText();
				        iWriter.println(TF_TIMER_PERIOD.getText());
					pWriter.println(text_to_save);                                         
                                              
                                        text_to_save = "#define CAL_BAT_V " + TF_CAL_BAT_V.getText();
				        iWriter.println(TF_CAL_BAT_V.getText());
					pWriter.println(text_to_save); 
                                              
                                        text_to_save = "#define CAL_V " + TF_CAL_V.getText();
				        iWriter.println(TF_CAL_V.getText());
					pWriter.println(text_to_save);                                         
                                              
                                        text_to_save = "#define CAL_I " + TF_CAL_I.getText();
				        iWriter.println(TF_CAL_I.getText());
					pWriter.println(text_to_save);  
                                              
                                        text_to_save = "#define INDUCTANCE " + TF_INDUCTANCE.getText();
				        iWriter.println(TF_INDUCTANCE.getText());
					pWriter.println(text_to_save); 
                                              
                                        text_to_save = "#define RESISTANCE " + TF_RESISTANCE.getText();
				        iWriter.println(TF_RESISTANCE.getText());
					pWriter.println(text_to_save); 
                                               
                                        text_to_save = "#define FLUX_LINKAGE " + TF_FLUX_LINKAGE.getText();
				        iWriter.println(TF_FLUX_LINKAGE.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define GAMMA " + TF_GAMMA.getText();
				        iWriter.println(TF_GAMMA.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define BATTERY_LEVEL_1 " + TF_BATTERY_LEVEL_1.getText();
				        iWriter.println(TF_BATTERY_LEVEL_1.getText());
					pWriter.println(text_to_save);                                        
                                               
                                        text_to_save = "#define BATTERY_LEVEL_2 " + TF_BATTERY_LEVEL_2.getText();
				        iWriter.println(TF_BATTERY_LEVEL_2.getText());
					pWriter.println(text_to_save); 
                                               
                                        text_to_save = "#define BATTERY_LEVEL_3 " + TF_BATTERY_LEVEL_3.getText();
				        iWriter.println(TF_BATTERY_LEVEL_3.getText());
					pWriter.println(text_to_save); 
                                               
                                        text_to_save = "#define BATTERY_LEVEL_4 " + TF_BATTERY_LEVEL_4.getText();
				        iWriter.println(TF_BATTERY_LEVEL_4.getText());
					pWriter.println(text_to_save); 
                                               
                                        text_to_save = "#define BATTERY_LEVEL_5 " + TF_BATTERY_LEVEL_5.getText();
				        iWriter.println(TF_BATTERY_LEVEL_5.getText());
					pWriter.println(text_to_save); 
                                               
                                        text_to_save = "#define P_FACTOR_I_Q " + TF_P_FACTOR_I_Q.getText();
				        iWriter.println(TF_P_FACTOR_I_Q.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define I_FACTOR_I_Q " + TF_I_FACTOR_I_Q.getText();
				        iWriter.println(TF_I_FACTOR_I_Q.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define P_FACTOR_I_D " + TF_P_FACTOR_I_D.getText();
				        iWriter.println(TF_P_FACTOR_I_D.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define I_FACTOR_I_D " + TF_I_FACTOR_I_D.getText();
				        iWriter.println(TF_I_FACTOR_I_D.getText());
					pWriter.println(text_to_save);   
                                                                         
                                        text_to_save = "#define P_FACTOR_PLL " + TF_P_FACTOR_PLL.getText();
				        iWriter.println(TF_P_FACTOR_PLL.getText());
					pWriter.println(text_to_save); 
                                        
                                        text_to_save = "#define I_FACTOR_PLL " + TF_I_FACTOR_PLL.getText();
				        iWriter.println(TF_I_FACTOR_PLL.getText());
					pWriter.println(text_to_save);  
                                        
                                        text_to_save = "#define P_FACTOR_SPEED " + TF_P_FACTOR_SPEED.getText();
				        iWriter.println(TF_P_FACTOR_SPEED.getText());
					pWriter.println(text_to_save); 
                                        
                                        text_to_save = "#define I_FACTOR_SPEED " + TF_I_FACTOR_SPEED.getText();
				        iWriter.println(TF_I_FACTOR_SPEED.getText());
					pWriter.println(text_to_save);     

                                        text_to_save = "#define SPDSHFT " + TF_SPDSHFT.getText();
				        iWriter.println(TF_SPDSHFT.getText());
					pWriter.println(text_to_save); 
                                        
                                        text_to_save = "#define SPEEDFILTER " + TF_SPEEDFILTER.getText();
				        iWriter.println(TF_SPEEDFILTER.getText());
					pWriter.println(text_to_save);                                          

                                        text_to_save = "#define SIXSTEPTHRESHOLD " + TF_SIXSTEPTHRESHOLD.getText();
				        iWriter.println(TF_SIXSTEPTHRESHOLD.getText());
					pWriter.println(text_to_save);                                         
                                        
                                        text_to_save = "#define TS_COEF " + TF_TS_COEF.getText();
				        iWriter.println(TF_TS_COEF.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define PAS_TIMEOUT " + TF_PAS_TIMEOUT.getText();
				        iWriter.println(TF_PAS_TIMEOUT.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define RAMP_END " + TF_PAS_RAMP_END.getText();
				        iWriter.println(TF_PAS_RAMP_END.getText());
					pWriter.println(text_to_save);

                                        text_to_save = "#define FRAC_HIGH " + TF_FRAC_HIGH.getText();
				        iWriter.println(TF_FRAC_HIGH.getText());
					pWriter.println(text_to_save);

                                        text_to_save = "#define FRAC_LOW " + TF_FRAC_LOW.getText();
				        iWriter.println(TF_FRAC_LOW.getText());
					pWriter.println(text_to_save);
                                        
                                        text_to_save = "#define THROTTLE_OFFSET " + TF_THROTTLE_OFFSET.getText();
				        iWriter.println(TF_THROTTLE_OFFSET.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define THROTTLE_MAX " + TF_THROTTLE_MAX.getText();
				        iWriter.println(TF_THROTTLE_MAX.getText());
					pWriter.println(text_to_save);

                                        text_to_save = "#define PUSHASSIST_CURRENT " + TF_PUSH_ASSIST .getText();
				        iWriter.println(TF_PUSH_ASSIST.getText());
					pWriter.println(text_to_save);                                        
                                               
                                        text_to_save = "#define WHEEL_CIRCUMFERENCE " + TF_WHEEL_CIRC.getText();
				        iWriter.println(TF_WHEEL_CIRC.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define GEAR_RATIO " + TF_GEAR_RATIO.getText();
				        iWriter.println(TF_GEAR_RATIO.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define SPEEDLIMIT " + TF_SPEED_LIMIT.getText();
				        iWriter.println(TF_SPEED_LIMIT.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define PULSES_PER_REVOLUTION " + TF_PULSES_PER_REVOLUTION.getText();
				        iWriter.println(TF_PULSES_PER_REVOLUTION.getText());
					pWriter.println(text_to_save);
                                               
                                        text_to_save = "#define PH_CURRENT_MAX " + TF_PH_CURRENT_MAX.getText();
				        iWriter.println(TF_PH_CURRENT_MAX.getText());
					pWriter.println(text_to_save);

                                        text_to_save = "#define BATTERYCURRENT_MAX " + TF_BATTERY_CURRENT_MAX.getText();
				        iWriter.println(TF_BATTERY_CURRENT_MAX.getText());
					pWriter.println(text_to_save);
                                        
                                        text_to_save = "#define REGEN_CURRENT " + TF_REGEN_CURRENT.getText();
				        iWriter.println(TF_REGEN_CURRENT.getText());
					pWriter.println(text_to_save);

                                        text_to_save = "#define REGEN_CURRENT_MAX " + TF_REGEN_CURRENT_MAX.getText();
				        iWriter.println(TF_REGEN_CURRENT_MAX.getText());
					pWriter.println(text_to_save);                                        

                                        text_to_save = "#define VOLTAGE_MIN " + TF_Voltage_MIN.getText();
				        iWriter.println(TF_Voltage_MIN.getText());
					pWriter.println(text_to_save);
                                        
                                        text_to_save = "#define VOLTAGE_MAX " + TF_Voltage_MAX.getText();
				        iWriter.println(TF_Voltage_MAX.getText());
					pWriter.println(text_to_save);                                        
                                        
                                        text_to_save = "#define SPEC_ANGLE " + TF_SPEC_ANGLE.getText();
				        iWriter.println(TF_SPEC_ANGLE.getText());
					pWriter.println(text_to_save);  
                                        
                                        if (RB_DIRDET.isSelected()) {
						text_to_save = "#define DIRDET";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_DIRDET.isSelected());
                                        
                                        if (RB_TORQUESENSOR.isSelected()) {
						text_to_save = "#define TS_MODE";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_TORQUESENSOR.isSelected());
                                        
                                        if (RB_TQ_ON_AD1.isSelected()) {
						text_to_save = "#define TQONAD1";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_TQ_ON_AD1.isSelected());
                                        
                                        if (RB_JLCD.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_618U //J-LCD";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_JLCD.isSelected());
                                        
                                        if (RB_KM5S.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_901U //KM5S";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_KM5S.isSelected());
                                        
                                        if (RB_EBICS.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_EBiCS";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_EBICS.isSelected());                                       
                                        
                                        if (RB_KUNTENG.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_KUNTENG //Kunteng LCD3/5 etc.";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_KUNTENG.isSelected());                                        

                                        if (RB_BAFANG.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_BAFANG //Bafang Displays, including 'Blaupunkt' ";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_BAFANG.isSelected());
                                        
                                        if (RB_DEBUG.isSelected()) {
						text_to_save = "#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG //ASCII Printout for debugging";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_DEBUG.isSelected());                                        
                                        
                                        if (RB_DISABLE_DYN_ADC.isSelected()) {
						text_to_save = "#define DISABLE_DYNAMIC_ADC";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_DISABLE_DYN_ADC.isSelected());
                                        
                                        if (RB_FAST_LOOP_LOG.isSelected()) {
						text_to_save = "#define FAST_LOOP_LOG";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_FAST_LOOP_LOG.isSelected()); 
                                        
                                        if (RB_SPEED_PLL.isSelected()) {
						text_to_save = "#define SPEED_PLL";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_SPEED_PLL.isSelected()); 

                                        if (RB_SPEEDTHROTTLE.isSelected()) {
						text_to_save = "#define SPEEDTHROTTLE";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_SPEEDTHROTTLE.isSelected()); 
                                        
                                        if (RB_LINEARREGEN.isSelected()) {
						text_to_save = "#define ADC_BRAKE";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_LINEARREGEN.isSelected());                                         
                                        
                                        if (RB_SS_EXTERNAL.isSelected()) {
						text_to_save = "#define SPEEDSOURCE EXTERNAL";
						pWriter.println(text_to_save);
					}
                                        else{
                                                text_to_save = "#define SPEEDSOURCE INTERNAL";
						pWriter.println(text_to_save);
                                        }                                        
					iWriter.println(RB_SS_EXTERNAL.isSelected());  

                                        if (RB_AUTODETECT.isSelected()) {
						text_to_save = "#define AUTODETECT 1";
						pWriter.println(text_to_save);
					}
                                        else{
                                                text_to_save = "#define AUTODETECT 0";
						pWriter.println(text_to_save);
                                        }                                        
					iWriter.println(RB_AUTODETECT.isSelected());                                         
                                        
                                        if (RB_REVERSE.isSelected()) {
						text_to_save = "#define REVERSE -1";
						pWriter.println(text_to_save);
					}
                                        else{
                                                text_to_save = "#define REVERSE 1";
						pWriter.println(text_to_save);
                                        }
					iWriter.println(RB_REVERSE.isSelected()); 

                                        if (RB_LIMIT_THR.isSelected()) {
						text_to_save = "#define LIMIT_THR";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_LIMIT_THR.isSelected());      
                                        
                                        if (RB_THR_OVERRIDE.isSelected()) {
						text_to_save = "#define THROTTLE_OVERRIDE";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_THR_OVERRIDE.isSelected());      
                                        
                                        if (RB_IND_PROFILE.isSelected()) {
						text_to_save = "#define INDIVIDUAL_MODES";
						pWriter.println(text_to_save);
					}
					iWriter.println(RB_IND_PROFILE.isSelected());                                         
                                        
                                        pWriter.println("\r\n#endif /* CONFIG_H_ */");
                                        
                                        iWriter.println(TF_PATH_ECLIPSE.getText());
                                        iWriter.println(TF_PATH_STM32_UTILITY.getText());


					iWriter.close();
 				} catch (IOException ioe) {
					ioe.printStackTrace();
				} finally {
					if (pWriter != null) {
						pWriter.flush();
						pWriter.close();

					}
				}  
                                try {
                                    String Flashoption;
				                    
                                    Flashoption = "STLink";
						
                                    	
                                    Process process = Runtime.getRuntime().exec("cmd /c start Start_Compiling " + TF_PATH_ECLIPSE.getText() + " "+ TF_PATH_STM32_UTILITY.getText() + " "  + Flashoption + " void");
				} catch (IOException e1) {
					TF_TS_COEF.setText("Error");
					e1.printStackTrace();
				}
          
          }
          
          
          });
                  
         		if (lastSettingsFile != null) {
			try {
				loadSettings(lastSettingsFile);
			} catch (Exception ex) {

			}
			provenSettingsList.clearSelection();
			experimentalSettingsList.clearSelection();
			//updateDependiencies(false);
		}
    }
    

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        BG_DISPLAYS = new javax.swing.ButtonGroup();
        BG_FLASHOPTIONS = new javax.swing.ButtonGroup();
        jTabbedPane1 = new javax.swing.JTabbedPane();
        TAB1 = new javax.swing.JPanel();
        TF_TS_COEF = new javax.swing.JTextField();
        Label_Parameter1 = new javax.swing.JLabel();
        RB_JLCD = new javax.swing.JRadioButton();
        RB_KM5S = new javax.swing.JRadioButton();
        RB_BAFANG = new javax.swing.JRadioButton();
        RB_KUNTENG = new javax.swing.JRadioButton();
        RB_DEBUG = new javax.swing.JRadioButton();
        RB_TORQUESENSOR = new javax.swing.JRadioButton();
        jLabel21 = new javax.swing.JLabel();
        jLabel27 = new javax.swing.JLabel();
        jLabel28 = new javax.swing.JLabel();
        jLabel29 = new javax.swing.JLabel();
        jLabel30 = new javax.swing.JLabel();
        TF_PAS_TIMEOUT = new javax.swing.JTextField();
        TF_PAS_RAMP_END = new javax.swing.JTextField();
        jLabel31 = new javax.swing.JLabel();
        jLabel32 = new javax.swing.JLabel();
        jLabel33 = new javax.swing.JLabel();
        TF_THROTTLE_OFFSET = new javax.swing.JTextField();
        TF_THROTTLE_MAX = new javax.swing.JTextField();
        jLabel34 = new javax.swing.JLabel();
        jLabel35 = new javax.swing.JLabel();
        jLabel36 = new javax.swing.JLabel();
        jLabel37 = new javax.swing.JLabel();
        jLabel38 = new javax.swing.JLabel();
        jLabel39 = new javax.swing.JLabel();
        jLabel40 = new javax.swing.JLabel();
        TF_WHEEL_CIRC = new javax.swing.JTextField();
        TF_GEAR_RATIO = new javax.swing.JTextField();
        TF_SPEED_LIMIT = new javax.swing.JTextField();
        TF_PULSES_PER_REVOLUTION = new javax.swing.JTextField();
        TF_PH_CURRENT_MAX = new javax.swing.JTextField();
        TF_SPEC_ANGLE = new javax.swing.JTextField();
        jLabel41 = new javax.swing.JLabel();
        jLabel42 = new javax.swing.JLabel();
        TF_PATH_ECLIPSE = new javax.swing.JTextField();
        TF_PATH_STM32_UTILITY = new javax.swing.JTextField();
        RB_REVERSE = new javax.swing.JRadioButton();
        jLabel44 = new javax.swing.JLabel();
        RB_DIRDET = new javax.swing.JRadioButton();
        TF_FRAC_HIGH = new javax.swing.JTextField();
        jLabel45 = new javax.swing.JLabel();
        TF_FRAC_LOW = new javax.swing.JTextField();
        jLabel46 = new javax.swing.JLabel();
        RB_EBICS = new javax.swing.JRadioButton();
        RB_TQ_ON_AD1 = new javax.swing.JRadioButton();
        jLabel47 = new javax.swing.JLabel();
        TF_Voltage_MIN = new javax.swing.JTextField();
        jLabel48 = new javax.swing.JLabel();
        TF_REGEN_CURRENT = new javax.swing.JTextField();
        jLabel49 = new javax.swing.JLabel();
        TF_PUSH_ASSIST = new javax.swing.JTextField();
        TF_BATTERY_CURRENT_MAX = new javax.swing.JTextField();
        jLabel50 = new javax.swing.JLabel();
        jLabel51 = new javax.swing.JLabel();
        TF_REGEN_CURRENT_MAX = new javax.swing.JTextField();
        jLabel52 = new javax.swing.JLabel();
        TF_Voltage_MAX = new javax.swing.JTextField();
        RB_THR_OVERRIDE = new javax.swing.JRadioButton();
        RB_IND_PROFILE = new javax.swing.JRadioButton();
        RB_LIMIT_THR = new javax.swing.JRadioButton();
        TAB2 = new javax.swing.JPanel();
        TF_TRIGGER_OFFSET = new javax.swing.JTextField();
        Label_Param3 = new javax.swing.JLabel();
        RB_FAST_LOOP_LOG = new javax.swing.JRadioButton();
        RB_DISABLE_DYN_ADC = new javax.swing.JRadioButton();
        TF_TRIGGER_DEFAULT = new javax.swing.JTextField();
        jLabel3 = new javax.swing.JLabel();
        jLabel4 = new javax.swing.JLabel();
        jLabel5 = new javax.swing.JLabel();
        TF_TIMER_PERIOD = new javax.swing.JTextField();
        jLabel6 = new javax.swing.JLabel();
        jLabel7 = new javax.swing.JLabel();
        TF_CAL_BAT_V = new javax.swing.JTextField();
        TF_CAL_V = new javax.swing.JTextField();
        jLabel8 = new javax.swing.JLabel();
        TF_CAL_I = new javax.swing.JTextField();
        jLabel9 = new javax.swing.JLabel();
        jLabel10 = new javax.swing.JLabel();
        jLabel11 = new javax.swing.JLabel();
        TF_INDUCTANCE = new javax.swing.JTextField();
        jLabel12 = new javax.swing.JLabel();
        TF_RESISTANCE = new javax.swing.JTextField();
        jLabel13 = new javax.swing.JLabel();
        TF_FLUX_LINKAGE = new javax.swing.JTextField();
        jLabel14 = new javax.swing.JLabel();
        TF_GAMMA = new javax.swing.JTextField();
        jLabel15 = new javax.swing.JLabel();
        jLabel16 = new javax.swing.JLabel();
        jLabel17 = new javax.swing.JLabel();
        jLabel18 = new javax.swing.JLabel();
        jLabel19 = new javax.swing.JLabel();
        TF_BATTERY_LEVEL_1 = new javax.swing.JTextField();
        TF_BATTERY_LEVEL_2 = new javax.swing.JTextField();
        jLabel20 = new javax.swing.JLabel();
        TF_BATTERY_LEVEL_3 = new javax.swing.JTextField();
        TF_BATTERY_LEVEL_4 = new javax.swing.JTextField();
        TF_BATTERY_LEVEL_5 = new javax.swing.JTextField();
        jLabel22 = new javax.swing.JLabel();
        jLabel23 = new javax.swing.JLabel();
        jLabel24 = new javax.swing.JLabel();
        jLabel25 = new javax.swing.JLabel();
        jLabel26 = new javax.swing.JLabel();
        TF_P_FACTOR_I_Q = new javax.swing.JTextField();
        TF_I_FACTOR_I_Q = new javax.swing.JTextField();
        TF_P_FACTOR_I_D = new javax.swing.JTextField();
        TF_I_FACTOR_I_D = new javax.swing.JTextField();
        RB_SPEED_PLL = new javax.swing.JRadioButton();
        jLabel53 = new javax.swing.JLabel();
        TF_P_FACTOR_PLL = new javax.swing.JTextField();
        jLabel54 = new javax.swing.JLabel();
        TF_I_FACTOR_PLL = new javax.swing.JTextField();
        RB_SPEEDTHROTTLE = new javax.swing.JRadioButton();
        RB_AUTODETECT = new javax.swing.JRadioButton();
        jLabel55 = new javax.swing.JLabel();
        TF_P_FACTOR_SPEED = new javax.swing.JTextField();
        TF_I_FACTOR_SPEED = new javax.swing.JTextField();
        jLabel56 = new javax.swing.JLabel();
        RB_SS_EXTERNAL = new javax.swing.JRadioButton();
        jLabel57 = new javax.swing.JLabel();
        TF_SPEEDFILTER = new javax.swing.JTextField();
        jLabel58 = new javax.swing.JLabel();
        TF_SIXSTEPTHRESHOLD = new javax.swing.JTextField();
        RB_LINEARREGEN = new javax.swing.JRadioButton();
        TF_SPDSHFT = new javax.swing.JTextField();
        jLabel59 = new javax.swing.JLabel();
        label1 = new java.awt.Label();
        jScrollPane1 = new javax.swing.JScrollPane();
        expSet = new javax.swing.JList<>();
        jLabel1 = new javax.swing.JLabel();
        jScrollPane2 = new javax.swing.JScrollPane();
        provSet = new javax.swing.JList<>();
        jLabel2 = new javax.swing.JLabel();
        jButton1 = new javax.swing.JButton();
        jButton2 = new javax.swing.JButton();
        jLabel43 = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setPreferredSize(new java.awt.Dimension(1250, 750));

        TF_TS_COEF.setText("60000");

        Label_Parameter1.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        Label_Parameter1.setLabelFor(TF_TS_COEF);
        Label_Parameter1.setText("Display");

        BG_DISPLAYS.add(RB_JLCD);
        RB_JLCD.setText("J-LCD");
        RB_JLCD.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                RB_JLCDActionPerformed(evt);
            }
        });

        BG_DISPLAYS.add(RB_KM5S);
        RB_KM5S.setText("KM5S");

        BG_DISPLAYS.add(RB_BAFANG);
        RB_BAFANG.setText("Bafang");

        BG_DISPLAYS.add(RB_KUNTENG);
        RB_KUNTENG.setText("Kunteng");

        BG_DISPLAYS.add(RB_DEBUG);
        RB_DEBUG.setText("Debug");

        RB_TORQUESENSOR.setText("Torquesensor");

        jLabel21.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jLabel21.setText("Ride Mode");

        jLabel27.setText("TS coefficient");

        jLabel28.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jLabel28.setText("PAS settings");

        jLabel29.setText("Timeout");

        jLabel30.setText("Ramp end");

        TF_PAS_TIMEOUT.setText("8000");

        TF_PAS_RAMP_END.setText("1600");

        jLabel31.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jLabel31.setText("Throttle settings");

        jLabel32.setText("Throttle offset");

        jLabel33.setText("Throttle max");

        TF_THROTTLE_OFFSET.setText("1255");

        TF_THROTTLE_MAX.setText("4096");

        jLabel34.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jLabel34.setText("Bike and Motor settings");

        jLabel35.setText("Wheel circumference");

        jLabel36.setText("Gear Ratio");

        jLabel37.setText("Speed Limit");

        jLabel38.setText("Pulses per revolution");

        jLabel39.setText("Motorcurrent max");

        jLabel40.setText("Motor specific angle");

        TF_WHEEL_CIRC.setText("2200");

        TF_GEAR_RATIO.setText("60");

        TF_SPEED_LIMIT.setText("25");

        TF_PULSES_PER_REVOLUTION.setText("1");

        TF_PH_CURRENT_MAX.setText("300");

        TF_SPEC_ANGLE.setText("-715827882LL");

        jLabel41.setText("Path to Eclipse");

        jLabel42.setText("Path to STM32 Utility");

        TF_PATH_ECLIPSE.setText("jTextField1");

        TF_PATH_STM32_UTILITY.setText("jTextField1");

        RB_REVERSE.setText("Reverse");

        jLabel44.setText("Motor direction");

        RB_DIRDET.setText("Direction Detection enabled");

        TF_FRAC_HIGH.setText("30");

        jLabel45.setText("Fraction upper limit");

        TF_FRAC_LOW.setText("15");

        jLabel46.setText("Fraction lower limit");

        BG_DISPLAYS.add(RB_EBICS);
        RB_EBICS.setText("EBiCS-Flutter");

        RB_TQ_ON_AD1.setText("Torque Signal on AD1");

        jLabel47.setText("Voltage min");

        TF_Voltage_MIN.setText("300");

        jLabel48.setText("Regencurrent");

        TF_REGEN_CURRENT.setText("300");

        jLabel49.setText("Push Assist");

        TF_PUSH_ASSIST.setText("30");
        TF_PUSH_ASSIST.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                TF_PUSH_ASSISTActionPerformed(evt);
            }
        });

        TF_BATTERY_CURRENT_MAX.setText("12000");
        TF_BATTERY_CURRENT_MAX.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                TF_BATTERY_CURRENT_MAXActionPerformed(evt);
            }
        });

        jLabel50.setText("Batterycurrent max");

        jLabel51.setText("Regencurrent max");

        TF_REGEN_CURRENT_MAX.setText("10000");

        jLabel52.setText("Voltage max");

        TF_Voltage_MAX.setText("2000");

        RB_THR_OVERRIDE.setText("Throttle Override ");

        RB_IND_PROFILE.setText("Individual Profile");

        RB_LIMIT_THR.setText("Limit Throttle to 6 kph");

        javax.swing.GroupLayout TAB1Layout = new javax.swing.GroupLayout(TAB1);
        TAB1.setLayout(TAB1Layout);
        TAB1Layout.setHorizontalGroup(
            TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB1Layout.createSequentialGroup()
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(jLabel41)
                            .addComponent(jLabel42))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(TF_PATH_STM32_UTILITY, javax.swing.GroupLayout.PREFERRED_SIZE, 386, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(TF_PATH_ECLIPSE, javax.swing.GroupLayout.PREFERRED_SIZE, 386, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(TAB1Layout.createSequentialGroup()
                                .addGap(25, 25, 25)
                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addGroup(TAB1Layout.createSequentialGroup()
                                        .addComponent(RB_DEBUG)
                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB1Layout.createSequentialGroup()
                                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addGroup(TAB1Layout.createSequentialGroup()
                                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addComponent(RB_JLCD)
                                                    .addComponent(RB_KM5S)
                                                    .addComponent(RB_KUNTENG)
                                                    .addComponent(RB_BAFANG)
                                                    .addComponent(RB_EBICS)
                                                    .addComponent(Label_Parameter1))
                                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 202, Short.MAX_VALUE)
                                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                                        .addGroup(TAB1Layout.createSequentialGroup()
                                                            .addComponent(jLabel46)
                                                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                                            .addComponent(TF_FRAC_LOW, javax.swing.GroupLayout.PREFERRED_SIZE, 49, javax.swing.GroupLayout.PREFERRED_SIZE))
                                                        .addGroup(TAB1Layout.createSequentialGroup()
                                                            .addComponent(jLabel45)
                                                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                                            .addComponent(TF_FRAC_HIGH, javax.swing.GroupLayout.PREFERRED_SIZE, 49, javax.swing.GroupLayout.PREFERRED_SIZE))
                                                        .addComponent(RB_DIRDET)
                                                        .addGroup(TAB1Layout.createSequentialGroup()
                                                            .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                                .addComponent(jLabel30)
                                                                .addComponent(jLabel29))
                                                            .addGap(18, 18, 18)
                                                            .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                                                .addComponent(TF_PAS_TIMEOUT)
                                                                .addComponent(TF_PAS_RAMP_END, javax.swing.GroupLayout.PREFERRED_SIZE, 64, javax.swing.GroupLayout.PREFERRED_SIZE))))
                                                    .addComponent(jLabel28)))
                                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB1Layout.createSequentialGroup()
                                                .addGap(0, 0, Short.MAX_VALUE)
                                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB1Layout.createSequentialGroup()
                                                        .addComponent(jLabel47)
                                                        .addGap(57, 57, 57)
                                                        .addComponent(TF_Voltage_MIN, javax.swing.GroupLayout.PREFERRED_SIZE, 49, javax.swing.GroupLayout.PREFERRED_SIZE))
                                                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB1Layout.createSequentialGroup()
                                                        .addComponent(jLabel52)
                                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                                        .addComponent(TF_Voltage_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, 49, javax.swing.GroupLayout.PREFERRED_SIZE)))))
                                        .addGap(44, 44, 44)
                                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addComponent(jLabel31)
                                            .addComponent(jLabel44)
                                            .addComponent(RB_REVERSE)
                                            .addGroup(TAB1Layout.createSequentialGroup()
                                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addComponent(jLabel32)
                                                    .addComponent(jLabel33)
                                                    .addComponent(jLabel49))
                                                .addGap(18, 18, 18)
                                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addComponent(TF_PUSH_ASSIST, javax.swing.GroupLayout.PREFERRED_SIZE, 57, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                    .addComponent(TF_THROTTLE_OFFSET, javax.swing.GroupLayout.PREFERRED_SIZE, 57, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                    .addComponent(TF_THROTTLE_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, 57, javax.swing.GroupLayout.PREFERRED_SIZE)))
                                            .addComponent(RB_THR_OVERRIDE)
                                            .addComponent(RB_IND_PROFILE)
                                            .addComponent(RB_LIMIT_THR))
                                        .addGap(33, 33, 33))))
                            .addGroup(TAB1Layout.createSequentialGroup()
                                .addContainerGap()
                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addComponent(jLabel21)
                                    .addGroup(TAB1Layout.createSequentialGroup()
                                        .addComponent(jLabel27, javax.swing.GroupLayout.PREFERRED_SIZE, 84, javax.swing.GroupLayout.PREFERRED_SIZE)
                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                        .addComponent(TF_TS_COEF, javax.swing.GroupLayout.PREFERRED_SIZE, 69, javax.swing.GroupLayout.PREFERRED_SIZE))
                                    .addGroup(TAB1Layout.createSequentialGroup()
                                        .addComponent(RB_TORQUESENSOR)
                                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                                        .addComponent(RB_TQ_ON_AD1)))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(jLabel34)
                            .addGroup(TAB1Layout.createSequentialGroup()
                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addComponent(jLabel36)
                                    .addComponent(jLabel35)
                                    .addComponent(jLabel37)
                                    .addComponent(jLabel38)
                                    .addComponent(jLabel40)
                                    .addComponent(jLabel39)
                                    .addComponent(jLabel50)
                                    .addComponent(jLabel48)
                                    .addComponent(jLabel51))
                                .addGap(6, 6, 6)
                                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addComponent(TF_REGEN_CURRENT_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_REGEN_CURRENT, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_WHEEL_CIRC, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_GEAR_RATIO, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_SPEED_LIMIT, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_PULSES_PER_REVOLUTION, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_SPEC_ANGLE, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_PH_CURRENT_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(TF_BATTERY_CURRENT_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE))))))
                .addGap(73, 73, 73))
        );
        TAB1Layout.setVerticalGroup(
            TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(TAB1Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(jLabel21)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(RB_TORQUESENSOR)
                    .addComponent(RB_TQ_ON_AD1))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(TF_TS_COEF, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel27))
                .addGap(30, 30, 30))
            .addGroup(TAB1Layout.createSequentialGroup()
                .addGap(22, 22, 22)
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(Label_Parameter1)
                        .addComponent(jLabel28))
                    .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                        .addComponent(jLabel31)
                        .addComponent(jLabel34)))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addComponent(RB_JLCD)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_KM5S)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_BAFANG)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_KUNTENG)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_EBICS, javax.swing.GroupLayout.PREFERRED_SIZE, 20, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_DEBUG)
                        .addGap(0, 0, Short.MAX_VALUE))
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel32)
                            .addComponent(TF_THROTTLE_OFFSET, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel33)
                            .addComponent(TF_THROTTLE_MAX, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel49)
                            .addComponent(TF_PUSH_ASSIST, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(jLabel44)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_REVERSE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_IND_PROFILE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addComponent(RB_THR_OVERRIDE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(RB_LIMIT_THR)
                        .addGap(30, 30, 30))
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel29)
                            .addComponent(TF_PAS_TIMEOUT, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel30)
                            .addComponent(TF_PAS_RAMP_END, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGap(15, 15, 15)
                        .addComponent(RB_DIRDET)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_FRAC_HIGH, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel45))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_FRAC_LOW, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel46))
                        .addGap(18, 18, 18)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel47)
                            .addComponent(TF_Voltage_MIN))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel52)
                            .addComponent(TF_Voltage_MAX))
                        .addGap(61, 61, 61))
                    .addGroup(TAB1Layout.createSequentialGroup()
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel35)
                            .addComponent(TF_WHEEL_CIRC))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(jLabel36)
                            .addComponent(TF_GEAR_RATIO))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(TF_SPEED_LIMIT)
                            .addComponent(jLabel37))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel38)
                            .addComponent(TF_PULSES_PER_REVOLUTION))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel40)
                            .addComponent(TF_SPEC_ANGLE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel39)
                            .addComponent(TF_PH_CURRENT_MAX))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel50)
                            .addComponent(TF_BATTERY_CURRENT_MAX))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel48)
                            .addComponent(TF_REGEN_CURRENT))
                        .addGap(4, 4, 4)
                        .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel51)
                            .addComponent(TF_REGEN_CURRENT_MAX))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)))
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.CENTER)
                    .addComponent(jLabel41)
                    .addComponent(TF_PATH_ECLIPSE, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(TAB1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.CENTER)
                    .addComponent(jLabel42)
                    .addComponent(TF_PATH_STM32_UTILITY, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        RB_JLCD.getAccessibleContext().setAccessibleName("RB_J-LCD");

        jTabbedPane1.addTab("Basic Settings", TAB1);

        TF_TRIGGER_OFFSET.setText("50");

        Label_Param3.setText("Trigger offset ADC");

        RB_FAST_LOOP_LOG.setText("enable fast loop logging");

        RB_DISABLE_DYN_ADC.setText("disable dynamic ADC");

        TF_TRIGGER_DEFAULT.setText("2020");

        jLabel3.setText("Trigger default");

        jLabel4.setText("ADC Timing");

        jLabel5.setText("Timer period");

        TF_TIMER_PERIOD.setText("2028");

        jLabel6.setText("Calibration");

        jLabel7.setText("Battery Voltage");

        TF_CAL_BAT_V.setText("256");

        TF_CAL_V.setText("15LL<<8");

        jLabel8.setText("FOC Voltage");

        TF_CAL_I.setText("38LL<<8");

        jLabel9.setText("FOC Current");

        jLabel10.setText("Sensorless settings");

        jLabel11.setText("Inductance");

        TF_INDUCTANCE.setText("6LL");

        jLabel12.setText("Resistance");

        TF_RESISTANCE.setText("40LL");

        jLabel13.setText("Flux Linkage");

        TF_FLUX_LINKAGE.setText("1200LL");

        jLabel14.setText("Gamma");

        TF_GAMMA.setText("9LL");

        jLabel15.setText("Battery settings");

        jLabel16.setText("Bar 1");

        jLabel17.setText("Bar 2");

        jLabel18.setText("Bar 3");

        jLabel19.setText("Bar 4");

        TF_BATTERY_LEVEL_1.setText("323000");

        TF_BATTERY_LEVEL_2.setText("329000");

        jLabel20.setText("Bar 5");

        TF_BATTERY_LEVEL_3.setText("344000");

        TF_BATTERY_LEVEL_4.setText("368000");

        TF_BATTERY_LEVEL_5.setText("380000");

        jLabel22.setText("PI control settings");

        jLabel23.setText("Iq p-factor");

        jLabel24.setText("Iq i-factor");

        jLabel25.setText("Id p-factor");

        jLabel26.setText("Id i-factor");

        TF_P_FACTOR_I_Q.setText("0.1L");

        TF_I_FACTOR_I_Q.setText("0.01L");

        TF_P_FACTOR_I_D.setText("1");

        TF_I_FACTOR_I_D.setText("1");

        RB_SPEED_PLL.setText("enable position PLL");

        jLabel53.setText("PLL p-factor");

        TF_P_FACTOR_PLL.setText("1");

        jLabel54.setText("PLL i-factor");

        TF_I_FACTOR_PLL.setText("1");

        RB_SPEEDTHROTTLE.setText("throttle controls speed");
        RB_SPEEDTHROTTLE.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                RB_SPEEDTHROTTLEActionPerformed(evt);
            }
        });

        RB_AUTODETECT.setText("enable autodetect in debug mode");
        RB_AUTODETECT.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                RB_AUTODETECTActionPerformed(evt);
            }
        });

        jLabel55.setText("speed  p-factor");

        TF_P_FACTOR_SPEED.setText("1");

        TF_I_FACTOR_SPEED.setText("1");

        jLabel56.setText("speed  i-factor");

        RB_SS_EXTERNAL.setText("use external speedsensor for speed limit");
        RB_SS_EXTERNAL.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                RB_SS_EXTERNALActionPerformed(evt);
            }
        });

        jLabel57.setText("speed filter");

        TF_SPEEDFILTER.setText("1");

        jLabel58.setText("6-step threshold");

        TF_SIXSTEPTHRESHOLD.setText("3500");

        RB_LINEARREGEN.setText("linear regen on AD2");
        RB_LINEARREGEN.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                RB_LINEARREGENActionPerformed(evt);
            }
        });

        TF_SPDSHFT.setText("1");

        jLabel59.setText("speedshift");

        javax.swing.GroupLayout TAB2Layout = new javax.swing.GroupLayout(TAB2);
        TAB2.setLayout(TAB2Layout);
        TAB2Layout.setHorizontalGroup(
            TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(TAB2Layout.createSequentialGroup()
                .addGap(27, 27, 27)
                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addComponent(RB_DISABLE_DYN_ADC)
                        .addGap(42, 42, 42)
                        .addComponent(RB_AUTODETECT)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addGroup(TAB2Layout.createSequentialGroup()
                                .addComponent(RB_FAST_LOOP_LOG)
                                .addGap(28, 28, 28)
                                .addComponent(RB_SS_EXTERNAL)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                .addComponent(jLabel59)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                                .addComponent(TF_SPDSHFT, javax.swing.GroupLayout.PREFERRED_SIZE, 47, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(TAB2Layout.createSequentialGroup()
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addGroup(TAB2Layout.createSequentialGroup()
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addComponent(jLabel4, javax.swing.GroupLayout.PREFERRED_SIZE, 159, javax.swing.GroupLayout.PREFERRED_SIZE)
                                            .addGroup(TAB2Layout.createSequentialGroup()
                                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                                        .addComponent(Label_Param3, javax.swing.GroupLayout.DEFAULT_SIZE, 93, Short.MAX_VALUE)
                                                        .addComponent(jLabel3, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                                                    .addComponent(jLabel5))
                                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                                    .addComponent(TF_TRIGGER_OFFSET)
                                                    .addComponent(TF_TRIGGER_DEFAULT, javax.swing.GroupLayout.DEFAULT_SIZE, 47, Short.MAX_VALUE)
                                                    .addComponent(TF_TIMER_PERIOD)))
                                            .addComponent(RB_LINEARREGEN))
                                        .addGap(25, 25, 25)
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addComponent(jLabel23, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel24, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel25, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel26, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel53, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel54, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel55, javax.swing.GroupLayout.Alignment.TRAILING)
                                            .addComponent(jLabel56, javax.swing.GroupLayout.Alignment.TRAILING))
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB2Layout.createSequentialGroup()
                                                .addGap(0, 0, Short.MAX_VALUE)
                                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addComponent(TF_I_FACTOR_SPEED, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                    .addGroup(TAB2Layout.createSequentialGroup()
                                                        .addGap(10, 10, 10)
                                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                            .addComponent(TF_P_FACTOR_PLL, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                            .addComponent(TF_I_FACTOR_PLL, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                            .addComponent(TF_P_FACTOR_SPEED, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)))))
                                            .addGroup(TAB2Layout.createSequentialGroup()
                                                .addGap(40, 40, 40)
                                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                    .addComponent(TF_P_FACTOR_I_Q, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)
                                                    .addComponent(TF_I_FACTOR_I_Q)
                                                    .addComponent(TF_P_FACTOR_I_D)))
                                            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, TAB2Layout.createSequentialGroup()
                                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                                .addComponent(TF_I_FACTOR_I_D, javax.swing.GroupLayout.PREFERRED_SIZE, 55, javax.swing.GroupLayout.PREFERRED_SIZE)))
                                        .addGap(31, 31, 31))
                                    .addGroup(TAB2Layout.createSequentialGroup()
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addGroup(TAB2Layout.createSequentialGroup()
                                                .addGap(190, 190, 190)
                                                .addComponent(jLabel22))
                                            .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                                .addComponent(RB_SPEEDTHROTTLE, javax.swing.GroupLayout.Alignment.TRAILING)
                                                .addComponent(RB_SPEED_PLL)))
                                        .addGap(107, 107, 107)))
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addGroup(TAB2Layout.createSequentialGroup()
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addComponent(jLabel11)
                                            .addComponent(jLabel12)
                                            .addComponent(jLabel13)
                                            .addComponent(jLabel14))
                                        .addGap(18, 18, 18)
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                            .addComponent(TF_RESISTANCE)
                                            .addComponent(TF_INDUCTANCE)
                                            .addComponent(TF_FLUX_LINKAGE)
                                            .addComponent(TF_GAMMA)))
                                    .addComponent(jLabel10, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                    .addGroup(TAB2Layout.createSequentialGroup()
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                            .addComponent(jLabel9, javax.swing.GroupLayout.PREFERRED_SIZE, 75, javax.swing.GroupLayout.PREFERRED_SIZE)
                                            .addComponent(jLabel7, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                            .addComponent(jLabel8, javax.swing.GroupLayout.PREFERRED_SIZE, 83, javax.swing.GroupLayout.PREFERRED_SIZE))
                                        .addGap(18, 18, 18)
                                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                                            .addComponent(TF_CAL_BAT_V)
                                            .addComponent(TF_CAL_V)
                                            .addComponent(TF_CAL_I, javax.swing.GroupLayout.PREFERRED_SIZE, 72, javax.swing.GroupLayout.PREFERRED_SIZE)))
                                    .addComponent(jLabel6, javax.swing.GroupLayout.PREFERRED_SIZE, 173, javax.swing.GroupLayout.PREFERRED_SIZE))))
                        .addGap(52, 52, 52)))
                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                        .addGroup(TAB2Layout.createSequentialGroup()
                            .addComponent(jLabel20)
                            .addGap(18, 18, 18)
                            .addComponent(TF_BATTERY_LEVEL_5))
                        .addComponent(jLabel15)
                        .addGroup(TAB2Layout.createSequentialGroup()
                            .addComponent(jLabel17)
                            .addGap(18, 18, 18)
                            .addComponent(TF_BATTERY_LEVEL_2))
                        .addGroup(TAB2Layout.createSequentialGroup()
                            .addComponent(jLabel18)
                            .addGap(18, 18, 18)
                            .addComponent(TF_BATTERY_LEVEL_3))
                        .addGroup(TAB2Layout.createSequentialGroup()
                            .addComponent(jLabel19)
                            .addGap(18, 18, 18)
                            .addComponent(TF_BATTERY_LEVEL_4))
                        .addGroup(TAB2Layout.createSequentialGroup()
                            .addComponent(jLabel16)
                            .addGap(18, 18, 18)
                            .addComponent(TF_BATTERY_LEVEL_1, javax.swing.GroupLayout.PREFERRED_SIZE, 90, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addComponent(jLabel58)
                            .addComponent(jLabel57))
                        .addGap(18, 18, 18)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                            .addComponent(TF_SPEEDFILTER)
                            .addComponent(TF_SIXSTEPTHRESHOLD, javax.swing.GroupLayout.DEFAULT_SIZE, 50, Short.MAX_VALUE))))
                .addGap(205, 205, 205))
        );
        TAB2Layout.setVerticalGroup(
            TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(TAB2Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel4)
                            .addComponent(jLabel22))
                        .addGap(14, 14, 14)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_TRIGGER_OFFSET, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(Label_Param3)
                            .addComponent(jLabel23)
                            .addComponent(TF_P_FACTOR_I_Q, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_TRIGGER_DEFAULT, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel3)
                            .addComponent(jLabel24)
                            .addComponent(TF_I_FACTOR_I_Q, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel5)
                            .addComponent(TF_TIMER_PERIOD, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel25)
                            .addComponent(TF_P_FACTOR_I_D, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel26)
                            .addComponent(TF_I_FACTOR_I_D, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGap(6, 6, 6)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel53)
                            .addComponent(TF_P_FACTOR_PLL, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(RB_SPEED_PLL))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel54)
                            .addComponent(TF_I_FACTOR_PLL, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel55)
                            .addComponent(TF_P_FACTOR_SPEED, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(RB_SPEEDTHROTTLE, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel56)
                            .addComponent(TF_I_FACTOR_SPEED, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(RB_LINEARREGEN, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                            .addGroup(TAB2Layout.createSequentialGroup()
                                .addComponent(jLabel10)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel11)
                                    .addComponent(TF_INDUCTANCE, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel12)
                                    .addComponent(TF_RESISTANCE, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel13)
                                    .addComponent(TF_FLUX_LINKAGE, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel14)
                                    .addComponent(TF_GAMMA, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addGap(10, 10, 10)
                                .addComponent(jLabel6))
                            .addGroup(javax.swing.GroupLayout.Alignment.LEADING, TAB2Layout.createSequentialGroup()
                                .addGap(9, 9, 9)
                                .addComponent(jLabel15)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel16)
                                    .addComponent(TF_BATTERY_LEVEL_1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel17)
                                    .addComponent(TF_BATTERY_LEVEL_2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel18)
                                    .addComponent(TF_BATTERY_LEVEL_3, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel19)
                                    .addComponent(TF_BATTERY_LEVEL_4, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                    .addComponent(jLabel20)
                                    .addComponent(TF_BATTERY_LEVEL_5, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jLabel7)
                            .addComponent(TF_CAL_BAT_V, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_CAL_V, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel8))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_CAL_I, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel9))))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 39, Short.MAX_VALUE)
                .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(RB_FAST_LOOP_LOG)
                            .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                                .addComponent(RB_SS_EXTERNAL, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addComponent(TF_SPDSHFT, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addComponent(jLabel59)))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(RB_DISABLE_DYN_ADC)
                            .addComponent(RB_AUTODETECT, javax.swing.GroupLayout.PREFERRED_SIZE, 23, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(TAB2Layout.createSequentialGroup()
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_SPEEDFILTER, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel57))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(TAB2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(TF_SIXSTEPTHRESHOLD, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel58))))
                .addGap(40, 40, 40))
        );

        jTabbedPane1.addTab("Advanced Settings", TAB2);

        label1.setFont(new java.awt.Font("Ebrima", 0, 24)); // NOI18N
        label1.setText("Lishui Parameter Configurator");

        expSet.setModel(new javax.swing.AbstractListModel<String>() {
            String[] strings = { "Item 1", "Item 2", "Item 3", "Item 4", "Item 5" };
            public int getSize() { return strings.length; }
            public String getElementAt(int i) { return strings[i]; }
        });
        jScrollPane1.setViewportView(expSet);

        jLabel1.setText("Experimental Settings");

        provSet.setModel(new javax.swing.AbstractListModel<String>() {
            String[] strings = { "Item 1", "Item 2", "Item 3", "Item 4", "Item 5" };
            public int getSize() { return strings.length; }
            public String getElementAt(int i) { return strings[i]; }
        });
        jScrollPane2.setViewportView(provSet);

        jLabel2.setText("Proven Settings");

        jButton1.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jButton1.setText("Compile & Flash");
        jButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton1ActionPerformed(evt);
            }
        });

        jButton2.setFont(new java.awt.Font("Tahoma", 1, 11)); // NOI18N
        jButton2.setText("Unlock controller");

        jLabel43.setText("     ");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(jTabbedPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 837, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(63, 63, 63)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(jScrollPane2)
                            .addComponent(jScrollPane1)
                            .addGroup(layout.createSequentialGroup()
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                                    .addComponent(jLabel1)
                                    .addComponent(jLabel2))
                                .addGap(0, 0, Short.MAX_VALUE))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(jButton1, javax.swing.GroupLayout.PREFERRED_SIZE, 129, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                                .addComponent(jButton2, javax.swing.GroupLayout.PREFERRED_SIZE, 136, javax.swing.GroupLayout.PREFERRED_SIZE))))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(label1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(jLabel43)))
                .addGap(29, 29, 29))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addGap(20, 20, 20)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(label1, javax.swing.GroupLayout.PREFERRED_SIZE, 46, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel43))
                .addGap(5, 5, 5)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(jTabbedPane1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(jLabel1)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(jScrollPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 109, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(jLabel2)
                        .addGap(3, 3, 3)
                        .addComponent(jScrollPane2, javax.swing.GroupLayout.PREFERRED_SIZE, 113, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(47, 47, 47)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(jButton1, javax.swing.GroupLayout.PREFERRED_SIZE, 50, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jButton2, javax.swing.GroupLayout.PREFERRED_SIZE, 47, javax.swing.GroupLayout.PREFERRED_SIZE))))
                .addContainerGap(279, Short.MAX_VALUE))
        );

        jTabbedPane1.getAccessibleContext().setAccessibleName("MotorConfiguration");

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void jButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton1ActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_jButton1ActionPerformed

    private void RB_JLCDActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_RB_JLCDActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_RB_JLCDActionPerformed

    private void TF_PUSH_ASSISTActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_TF_PUSH_ASSISTActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_TF_PUSH_ASSISTActionPerformed

    private void TF_BATTERY_CURRENT_MAXActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_TF_BATTERY_CURRENT_MAXActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_TF_BATTERY_CURRENT_MAXActionPerformed

    private void RB_SS_EXTERNALActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_RB_SS_EXTERNALActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_RB_SS_EXTERNALActionPerformed

    private void RB_AUTODETECTActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_RB_AUTODETECTActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_RB_AUTODETECTActionPerformed

    private void RB_SPEEDTHROTTLEActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_RB_SPEEDTHROTTLEActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_RB_SPEEDTHROTTLEActionPerformed

    private void RB_LINEARREGENActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_RB_LINEARREGENActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_RB_LINEARREGENActionPerformed

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
        try {
            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
                if ("Nimbus".equals(info.getName())) {
                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
                    break;
                }
            }
        } catch (ClassNotFoundException ex) {
            java.util.logging.Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (InstantiationException ex) {
            java.util.logging.Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (IllegalAccessException ex) {
            java.util.logging.Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(Lishui_Parameter_Configurator.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                new Lishui_Parameter_Configurator().setVisible(true);
            }
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.ButtonGroup BG_DISPLAYS;
    private javax.swing.ButtonGroup BG_FLASHOPTIONS;
    private javax.swing.JLabel Label_Param3;
    private javax.swing.JLabel Label_Parameter1;
    private javax.swing.JRadioButton RB_AUTODETECT;
    private javax.swing.JRadioButton RB_BAFANG;
    private javax.swing.JRadioButton RB_DEBUG;
    private javax.swing.JRadioButton RB_DIRDET;
    private javax.swing.JRadioButton RB_DISABLE_DYN_ADC;
    private javax.swing.JRadioButton RB_EBICS;
    private javax.swing.JRadioButton RB_FAST_LOOP_LOG;
    private javax.swing.JRadioButton RB_IND_PROFILE;
    private javax.swing.JRadioButton RB_JLCD;
    private javax.swing.JRadioButton RB_KM5S;
    private javax.swing.JRadioButton RB_KUNTENG;
    private javax.swing.JRadioButton RB_LIMIT_THR;
    private javax.swing.JRadioButton RB_LINEARREGEN;
    private javax.swing.JRadioButton RB_REVERSE;
    private javax.swing.JRadioButton RB_SPEEDTHROTTLE;
    private javax.swing.JRadioButton RB_SPEED_PLL;
    private javax.swing.JRadioButton RB_SS_EXTERNAL;
    private javax.swing.JRadioButton RB_THR_OVERRIDE;
    private javax.swing.JRadioButton RB_TORQUESENSOR;
    private javax.swing.JRadioButton RB_TQ_ON_AD1;
    private javax.swing.JPanel TAB1;
    private javax.swing.JPanel TAB2;
    private javax.swing.JTextField TF_BATTERY_CURRENT_MAX;
    private javax.swing.JTextField TF_BATTERY_LEVEL_1;
    private javax.swing.JTextField TF_BATTERY_LEVEL_2;
    private javax.swing.JTextField TF_BATTERY_LEVEL_3;
    private javax.swing.JTextField TF_BATTERY_LEVEL_4;
    private javax.swing.JTextField TF_BATTERY_LEVEL_5;
    private javax.swing.JTextField TF_CAL_BAT_V;
    private javax.swing.JTextField TF_CAL_I;
    private javax.swing.JTextField TF_CAL_V;
    private javax.swing.JTextField TF_FLUX_LINKAGE;
    private javax.swing.JTextField TF_FRAC_HIGH;
    private javax.swing.JTextField TF_FRAC_LOW;
    private javax.swing.JTextField TF_GAMMA;
    private javax.swing.JTextField TF_GEAR_RATIO;
    private javax.swing.JTextField TF_INDUCTANCE;
    private javax.swing.JTextField TF_I_FACTOR_I_D;
    private javax.swing.JTextField TF_I_FACTOR_I_Q;
    private javax.swing.JTextField TF_I_FACTOR_PLL;
    private javax.swing.JTextField TF_I_FACTOR_SPEED;
    private javax.swing.JTextField TF_PAS_RAMP_END;
    private javax.swing.JTextField TF_PAS_TIMEOUT;
    private javax.swing.JTextField TF_PATH_ECLIPSE;
    private javax.swing.JTextField TF_PATH_STM32_UTILITY;
    private javax.swing.JTextField TF_PH_CURRENT_MAX;
    private javax.swing.JTextField TF_PULSES_PER_REVOLUTION;
    private javax.swing.JTextField TF_PUSH_ASSIST;
    private javax.swing.JTextField TF_P_FACTOR_I_D;
    private javax.swing.JTextField TF_P_FACTOR_I_Q;
    private javax.swing.JTextField TF_P_FACTOR_PLL;
    private javax.swing.JTextField TF_P_FACTOR_SPEED;
    private javax.swing.JTextField TF_REGEN_CURRENT;
    private javax.swing.JTextField TF_REGEN_CURRENT_MAX;
    private javax.swing.JTextField TF_RESISTANCE;
    private javax.swing.JTextField TF_SIXSTEPTHRESHOLD;
    private javax.swing.JTextField TF_SPDSHFT;
    private javax.swing.JTextField TF_SPEC_ANGLE;
    private javax.swing.JTextField TF_SPEEDFILTER;
    private javax.swing.JTextField TF_SPEED_LIMIT;
    private javax.swing.JTextField TF_THROTTLE_MAX;
    private javax.swing.JTextField TF_THROTTLE_OFFSET;
    private javax.swing.JTextField TF_TIMER_PERIOD;
    private javax.swing.JTextField TF_TRIGGER_DEFAULT;
    private javax.swing.JTextField TF_TRIGGER_OFFSET;
    private javax.swing.JTextField TF_TS_COEF;
    private javax.swing.JTextField TF_Voltage_MAX;
    private javax.swing.JTextField TF_Voltage_MIN;
    private javax.swing.JTextField TF_WHEEL_CIRC;
    private javax.swing.JList<String> expSet;
    private javax.swing.JButton jButton1;
    private javax.swing.JButton jButton2;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel11;
    private javax.swing.JLabel jLabel12;
    private javax.swing.JLabel jLabel13;
    private javax.swing.JLabel jLabel14;
    private javax.swing.JLabel jLabel15;
    private javax.swing.JLabel jLabel16;
    private javax.swing.JLabel jLabel17;
    private javax.swing.JLabel jLabel18;
    private javax.swing.JLabel jLabel19;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel20;
    private javax.swing.JLabel jLabel21;
    private javax.swing.JLabel jLabel22;
    private javax.swing.JLabel jLabel23;
    private javax.swing.JLabel jLabel24;
    private javax.swing.JLabel jLabel25;
    private javax.swing.JLabel jLabel26;
    private javax.swing.JLabel jLabel27;
    private javax.swing.JLabel jLabel28;
    private javax.swing.JLabel jLabel29;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel30;
    private javax.swing.JLabel jLabel31;
    private javax.swing.JLabel jLabel32;
    private javax.swing.JLabel jLabel33;
    private javax.swing.JLabel jLabel34;
    private javax.swing.JLabel jLabel35;
    private javax.swing.JLabel jLabel36;
    private javax.swing.JLabel jLabel37;
    private javax.swing.JLabel jLabel38;
    private javax.swing.JLabel jLabel39;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel40;
    private javax.swing.JLabel jLabel41;
    private javax.swing.JLabel jLabel42;
    private javax.swing.JLabel jLabel43;
    private javax.swing.JLabel jLabel44;
    private javax.swing.JLabel jLabel45;
    private javax.swing.JLabel jLabel46;
    private javax.swing.JLabel jLabel47;
    private javax.swing.JLabel jLabel48;
    private javax.swing.JLabel jLabel49;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel50;
    private javax.swing.JLabel jLabel51;
    private javax.swing.JLabel jLabel52;
    private javax.swing.JLabel jLabel53;
    private javax.swing.JLabel jLabel54;
    private javax.swing.JLabel jLabel55;
    private javax.swing.JLabel jLabel56;
    private javax.swing.JLabel jLabel57;
    private javax.swing.JLabel jLabel58;
    private javax.swing.JLabel jLabel59;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JScrollPane jScrollPane1;
    private javax.swing.JScrollPane jScrollPane2;
    private javax.swing.JTabbedPane jTabbedPane1;
    private java.awt.Label label1;
    private javax.swing.JList<String> provSet;
    // End of variables declaration//GEN-END:variables
}
