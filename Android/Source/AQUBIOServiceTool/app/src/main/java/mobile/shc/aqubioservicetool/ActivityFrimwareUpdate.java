package mobile.shc.aqubioservicetool;

import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.app.Activity;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListAdapter;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;


public class ActivityFrimwareUpdate extends Activity {

    /***** file browser *****/
    // Stores names of traversed directories
    ArrayList<String> str = new ArrayList<String>();
    // Check if the first level of the directory structure is the one showing
    private Boolean firstLvl = true;
    private static final String TAG = "F_PATH";
    private Item[] fileList;
    private File path ;
    private String chosenFile;
    private static final int DIALOG_LOAD_FILE = 1000;
    ListAdapter adapter;
    byte[] fileContent;
    Button btnSet;
    TextView txtSSID, txtPassword;
    byte[] dataSIID = new byte[70];
    ProgressDialog progressDialog;
    int step = 0;
    Context cn = this;
    Thread socketServerThread;
    ServerSocket ss;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_frimware_update);


        path = new File(Environment.getExternalStorageDirectory().getPath());
        btnSet = (Button)findViewById(R.id.btn_Set);
        txtSSID = (TextView)findViewById(R.id.txt_SSID);
        txtPassword = (TextView)findViewById(R.id.txt_SSID_Pass);
        progressDialog = new ProgressDialog(this);
        progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
        progressDialog.setCancelable(false);
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
    }

    @Override
    protected void onResume() {
        super.onResume();
        try {
            registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at onResume : \n"+ ex.toString());
        }
    }@Override
    protected void onPause() {
        super.onPause();
        try {
            unregisterReceiver(mGattUpdateReceiver);
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at onPause : \n"+ ex.toString());
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (ss != null) {
            try {
                ss.close();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        try {
            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService = null;
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at onDestroy : \n"+ ex.toString());
        }
    }

    public int CheckInfoLogin(String strSSID,String strPass)
    {
        int ret = 0;
        if(strSSID.compareTo("") == 0 || strSSID.isEmpty() || strSSID.length() > 32)
            ret = 1;
        else if(strPass.compareTo("") == 0 || strPass.isEmpty() || strPass.length() > 32)
            ret = 2;
        return ret;
    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        intentFilter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        return intentFilter;
    }

    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.e("ACTION","ACTION_DATA_AVAILABLE");
                if (!((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.strData.isEmpty() &&
                        ((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.isFinish) {

                    if (((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.strData.compareTo("OK") == 0) {
                        ((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.isFinish = false;
                        //Switch to wifi and send flash file
                        if(step == 0)//send "E" command
                        {
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet.setValue(CreatePackageE());
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                            step = 1;
                        }
                        else if(step == 1)
                        {
                           //switch to wifi
                            Thread socketServerThread = new Thread(new SendFileFlash());
                            socketServerThread.start();
                        }

                    } else if (((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.strData.compareTo("NG") == 0) {
                       // wrong data
                        if(progressDialog.isShowing())
                            progressDialog.cancel();
                        //Toast.makeText(cn,"Have problem. Please try again",Toast.LENGTH_LONG).show();
                        ((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.isFinish = false;

                    } else if (((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.strData.compareTo("CK") == 0) {
                        Toast.makeText(getApplication(), "Data received error.", Toast.LENGTH_LONG).show();
                        ((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.isFinish = false;
                        // wrong data
                        if(progressDialog.isShowing()) {
                            progressDialog.cancel();
                            //Toast.makeText(cn, "Have problem. Please try again", Toast.LENGTH_LONG).show();
                        }
                    } else {
                        ((MyApplication) ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.isFinish = false;
                        if(progressDialog.isShowing()) {
                            progressDialog.cancel();
                            //Toast.makeText(cn, "Have problem. Please try again", Toast.LENGTH_LONG).show();
                        }
                    }
                }
            }
        }
    };

    public  void OnSet(View v)
    {
        progressDialog.setTitle(CHardCode.strTitleWaittingSend);
        progressDialog.setMessage(CHardCode.strWaittingSend);
        progressDialog.show();
        for(int i=0;i<4;i++)
        {
            switch (i)
            {
                case 0:
                case 1:
                case 2:
                    byte[] temp = new byte[20];
                    for(int j =0; j<20;j++)
                    {
                        temp[j] = dataSIID[(i*20)+j];
                    }
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet.setValue(temp);
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    break;
                case 3:
                    byte[] temp3 = new byte[10];
                    for(int j =0; j<10;j++)
                    {
                        temp3[j] = dataSIID[60+j];
                    }
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet.setValue(temp3);
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                    ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.characteristicSet);
                    break;
            }
        }

    }

    public class SendFileFlash extends Thread
    {
        @Override
        public void run() {
            try {
                Boolean end = false;
                ss = new ServerSocket(CHardCode.STR_PORT);
                while (!end) {
                    //Server is waiting for client here, if needed
                    Socket s = ss.accept();
                    DataOutputStream output = new DataOutputStream(s.getOutputStream()); //Autoflush
                    InputStream input = s.getInputStream();
                    byte[] revData = new byte[2];
                    input.read(revData);
                    if (new String(revData).compareTo("OK") == 0) {
                        output.write("update".getBytes(),0,"update".getBytes().length);
                        while(true)
                        {
                            input.read(revData);
                            if (new String(revData).compareTo("OK") == 0) {
                                break;
                            }
                        }
                        byte[] byteLenghts = new byte[4];

                        byteLenghts[0] = (byte) (fileContent.length >> 24);
                        byteLenghts[1] = (byte) (fileContent.length >> 16);
                        byteLenghts[2] = (byte) (fileContent.length >> 8);
                        byteLenghts[3] = (byte) (fileContent.length /*>> 0*/);
                        output.write(byteLenghts,0,byteLenghts.length);
                        while(true)
                        {
                            input.read(revData);
                            if (new String(revData).compareTo("OK") == 0) {
                                break;
                            }
                        }
                        while(true)
                        {
                            if(fileContent.length <=4096) {
                                output.write(fileContent, 0, fileContent.length);
                                break;
                            }
                            else
                            {
                                int blockSize = 4096;
                                int blockCount = (fileContent.length + blockSize - 1) / blockSize;
                                byte[] range = null;

                                for (int i = 1; i <= blockCount; i++) {
                                    int idx = (i - 1) * blockSize;
                                    range = Arrays.copyOfRange(fileContent, idx, idx + blockSize);
                                    output.write(range, 0, range.length);
                                    while(true)
                                    {
                                        input.read(revData);
                                        if (new String(revData).compareTo("OK") == 0) {
                                            break;
                                        }
                                    }
                                }
                                break;
                            }
                        }
                        s.close();
                        end = true;
                    }
                }
                ss.close();
                if (progressDialog.isShowing())
                    progressDialog.cancel();
                ActivityFrimwareUpdate.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(cn, "Finished.", Toast.LENGTH_LONG).show();
                    }
                });

            } catch (UnknownHostException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void ReadFile(View v)
    {
        int resultLogin = CheckInfoLogin(txtSSID.getText().toString(),txtPassword.getText().toString());
        if(resultLogin == 0) {
            loadFileList();
            showDialog(DIALOG_LOAD_FILE);
            CreateDataSSID();
        }
        else{
            switch (resultLogin)
            {
                case 1:
                    Toast.makeText(this, CHardCode.strErrID, Toast.LENGTH_LONG).show();
                    break;
                case 2:
                    Toast.makeText(this, CHardCode.strErrPass, Toast.LENGTH_LONG).show();
                    break;
            }
        }
    }

    public void BackHome(View v)
    {
        if(((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService != null)
        {
            ((MyApplication)ActivityFrimwareUpdate.this.getApplication()).mBluetoothLeService.disconnect();
        }
        Intent myIntent = new Intent(cn, ActivityLogin.class);
        startActivity(myIntent);
    }

    public void CreateDataSSID()
    {
        byte[] header = {0x40,0x46,0x31,0x32};
        byte[] ssid = new byte[32];
        byte[] pass = new byte[32];
        byte[] arrSSID = txtSSID.getText().toString().getBytes();
        for(int i=0; i<arrSSID.length;i++)
        {
            ssid[i] = arrSSID[i];
        }
        byte[] arrPass = txtPassword.getText().toString().getBytes();
        for(int i=0; i<arrPass.length;i++)
        {
            pass[i] = arrPass[i];
        }
        for (int i = 0; i < 68; ++i)
        {
            if(i<4){
                dataSIID[i] = header[i];
            }
            else if(i<36 && i > 3){
                dataSIID[i] = ssid[i-4];
            }
            else if(i<68 && i > 35){
                dataSIID[i] = pass[i-36];
            }
        }


        //check sum
        byte checkSum=0x00;
        for(int i = 0;i<68;i++){
            checkSum^=dataSIID[i];
        }
        dataSIID[68] = checkSum;
        dataSIID[69] = 0x0a;
    }

    //create package PIN
    public byte[] CreatePackageE()
    {
        byte checkSum=0x00;
        byte[] data = "E".getBytes();
        byte[] header = {0x40,0x06,0x31,0,0,0};
        header[3] = data[0];
        for(int i = 0;i<4;i++){
            checkSum^=header[i];
        }
        header[4] = checkSum;
        header[5] = 0x0a;
        return  header;
    }


    /***** File browser *****/
    private void loadFileList() {
        try {
            path.mkdirs();
        } catch (SecurityException e) {
            Log.e(TAG, "unable to write on the sd card ");
        }

        // Checks whether path exists
        if (path.exists()) {
            FilenameFilter filter = new FilenameFilter() {
                @Override
                public boolean accept(File dir, String filename) {
                    File sel = new File(dir, filename);
                    // Filters based on whether the file is hidden or not
                    return (sel.isFile() || sel.isDirectory())
                            && !sel.isHidden();

                }
            };
            String[] fList = path.list(filter);
            fileList = new Item[fList.length];
            for (int i = 0; i < fList.length; i++) {
                fileList[i] = new Item(fList[i], R.drawable.file_icon);

                // Convert into file path
                File sel = new File(path, fList[i]);

                // Set drawables
                if (sel.isDirectory()) {
                    fileList[i].icon = R.drawable.directory_icon;
                    Log.d("DIRECTORY", fileList[i].file);
                } else {
                    Log.d("FILE", fileList[i].file);
                }
            }

            if (!firstLvl) {
                Item temp[] = new Item[fileList.length + 1];
                for (int i = 0; i < fileList.length; i++) {
                    temp[i + 1] = fileList[i];
                }
                temp[0] = new Item("Up", R.drawable.directory_up);
                fileList = temp;
            }
        } else {
            Log.e(TAG, "path does not exist (sdcard)");
        }

        adapter = new ArrayAdapter<Item>(this,
                android.R.layout.select_dialog_item, android.R.id.text1,
                fileList) {
            @Override
            public View getView(int position, View convertView, ViewGroup parent) {
                // creates view
                View view = super.getView(position, convertView, parent);
                TextView textView = (TextView) view
                        .findViewById(android.R.id.text1);

                // put the image on the text view
                textView.setCompoundDrawablesWithIntrinsicBounds(
                        fileList[position].icon, 0, 0, 0);

                // add margin between image and text (support various screen
                // densities)
                int dp5 = (int) (5 * getResources().getDisplayMetrics().density + 0.5f);
                textView.setCompoundDrawablePadding(dp5);

                return view;
            }
        };

    }

    private class Item {
        public String file;
        public int icon;

        public Item(String file, Integer icon) {
            this.file = file;
            this.icon = icon;
        }

        @Override
        public String toString() {
            return file;
        }
    }

    @Override
    protected Dialog onCreateDialog(int id) {
        Dialog dialog = null;
        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        if (fileList == null) {
            Log.e(TAG, "No files loaded");
            dialog = builder.create();
            return dialog;
        }

        switch (id) {
            case DIALOG_LOAD_FILE:
                builder.setTitle("Choose your file");
                builder.setAdapter(adapter, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        chosenFile = fileList[which].file;
                        File sel = new File(path + "/" + chosenFile);
                        if (sel.isDirectory()) {
                            firstLvl = false;

                            // Adds chosen directory to list
                            str.add(chosenFile);
                            fileList = null;
                            path = new File(sel + "");

                            loadFileList();

                            removeDialog(DIALOG_LOAD_FILE);
                            showDialog(DIALOG_LOAD_FILE);
                            Log.d(TAG, path.getAbsolutePath());

                        }

                        // Checks if 'up' was clicked
                        else if (chosenFile.equalsIgnoreCase("up") && !sel.exists()) {

                            // present directory removed from list
                            String s = str.remove(str.size() - 1);

                            // path modified to exclude present directory
                            path = new File(path.toString().substring(0,
                                    path.toString().lastIndexOf(s)));
                            fileList = null;

                            // if there are no more directories in the list, then
                            // its the first level
                            if (str.isEmpty()) {
                                firstLvl = true;
                            }
                            loadFileList();

                            removeDialog(DIALOG_LOAD_FILE);
                            showDialog(DIALOG_LOAD_FILE);
                            Log.d(TAG, path.getAbsolutePath());

                        }
                        // File picked
                        else {
                            // Perform action with file picked
                            Log.e("CLICK", sel.getPath());
                            fileContent = getByte(sel.getPath());
                        }

                    }
                });
                break;
        }
        dialog = builder.show();
        return dialog;
    }

    private byte[] getByte(String path) {
        byte[] getBytes = {};
        try {
            File file = new File(path);
            getBytes = new byte[(int) file.length()];
            InputStream is = new FileInputStream(file);
            is.read(getBytes);
            is.close();
            btnSet.setBackgroundResource(R.drawable.custom_button);
            btnSet.setEnabled(true);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return getBytes;
    }

}
