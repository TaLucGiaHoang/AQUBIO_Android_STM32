package mobile.shc.aqubioservicetool;

import android.Manifest;
import android.app.Activity;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

import static android.content.ContentValues.TAG;

public class ActivityLogin extends Activity {

    Context cn = this;
    boolean isSupport = true;
    BluetoothAdapter mBluetoothAdapter;
    List<BluetoothDevice> lsItemDevices;
    boolean mScanning;
    Handler mHandler;
    String strAddress;
    ProgressDialog progressDialog;
    boolean mConnected = false;
    TextView txtID, txtPass, txtName;


    // custom dialog
    Dialog dialog ;
    boolean isVerify = false;
    byte checkSum = 0x00;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login);
        dialog = new Dialog(this);
        //check support BLE
        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            Toast.makeText(this, CHardCode.strNotSupport, Toast.LENGTH_LONG).show();
            isSupport = false;
        } else {
            // Initializes Bluetooth adapter.
            if (Build.VERSION.SDK_INT >= 23) {
                // Marshmallow+ Permission APIs
                requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, CHardCode.PER_COARSE);
                requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, CHardCode.PER_FINE);
                requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 110);
                requestPermissions(new String[]{Manifest.permission.READ_EXTERNAL_STORAGE}, 110);
            }
            // Actually set it in response to ACTION_PAIRING_REQUEST.
            final IntentFilter pairingRequestFilter = new IntentFilter(BluetoothDevice.ACTION_PAIRING_REQUEST);
            pairingRequestFilter.setPriority(IntentFilter.SYSTEM_HIGH_PRIORITY - 1);
            registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
            final BluetoothManager bluetoothManager =
                    (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
            mBluetoothAdapter = bluetoothManager.getAdapter();
        }
        lsItemDevices = new ArrayList<BluetoothDevice>();
        mHandler = new Handler();

        txtID = (TextView)findViewById(R.id.txt_loginID);
        txtPass = (TextView)findViewById(R.id.txt_loginPass);
        txtName = (TextView)findViewById(R.id.txt_loginName);
    }

    @Override
    protected void onResume() {
        super.onResume();
        try {
            registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
            if (((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService != null) {
                final boolean result = ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.connect(strAddress);
                Log.e(TAG, "Connect request result=" + result);
            }
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
        try {
            unbindService(mServiceConnection);
            ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService = null;
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at onDestroy : \n"+ ex.toString());
        }
    }


    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case CHardCode.PER_COARSE: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // permission was granted, yay! Do the
                    // contacts-related task you need to do.
                    LocationManager locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);

                    if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)){
                        //Toast.makeText(this, "GPS is Enabled in your devide", Toast.LENGTH_SHORT).show();
                    }else{
                        Intent callGPSSettingIntent = new Intent(
                                android.provider.Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                        startActivity(callGPSSettingIntent);
                    }
                } else {

                    // permission denied, boo! Disable the
                    // functionality that depends on this permission.
                    Toast.makeText(this,"Permission denied.",Toast.LENGTH_LONG).show();
                }
                return;
            }

            case CHardCode.PER_FINE: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // permission was granted, yay! Do the
                    // contacts-related task you need to do.

                } else {

                    // permission denied, boo! Disable the
                    // functionality that depends on this permission.
                }
                return;
            }

            // other 'case' lines to check for other
            // permissions this app might request
        }
    }

    public void ExecuteTool(View v)
    {
        try {
            if (isSupport) {
                int resultLogin = CheckInfoLogin(txtID.getText().toString(),txtPass.getText().toString(),txtName.getText().toString());
                if(resultLogin != 0) {//hienvd
                    lsItemDevices.clear();
                    scanLeDevice(true);
                    progressDialog = new ProgressDialog(this);
                    progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
                    progressDialog.setCancelable(false);
                    progressDialog.setTitle(CHardCode.strTitleWaitting);

                    Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
                    bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);

                    progressDialog.setMessage(CHardCode.strWaitting);
                    progressDialog.show();
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
                        case 3:
                            Toast.makeText(this, CHardCode.strErrName, Toast.LENGTH_LONG).show();
                            break;
                    }
                }
            } else {
                Toast.makeText(this, CHardCode.strNotSupport, Toast.LENGTH_LONG).show();
            }
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at ScanDeviceBluetooth : \n"+ ex.toString());
        }
    }

    public int CheckInfoLogin(String strID,String strPass, String strName)
    {
        int ret = 0;
        if(strID.compareTo(CHardCode.STR_LOGIN_ID) != 0)
            ret = 1;
        else if(strPass.compareTo(CHardCode.STR_LOGIN_PASS) != 0)
            ret = 2;
        else if(strName.compareTo(CHardCode.STR_LOGIN_NAME) != 0)
            ret = 3;
        return ret;
    }

    private void scanLeDevice(final boolean enable) {
        try {
            if (enable) {
                // Stops scanning after a pre-defined scan period.
                mHandler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        mScanning = false;
                        mBluetoothAdapter.stopLeScan(mLeScanCallback);
                        if(strAddress == "" || strAddress == null){
                            if(progressDialog.isShowing())
                                progressDialog.dismiss();
                            Toast.makeText(cn,CHardCode.strTimeout,Toast.LENGTH_LONG).show();
                        }
                    }
                }, CHardCode.SCAN_PERIOD);

                mScanning = true;
                mBluetoothAdapter.startLeScan(mLeScanCallback);
            } else {
                mScanning = false;
                mBluetoothAdapter.stopLeScan(mLeScanCallback);
                if(strAddress == "" || strAddress == null){
                    if(progressDialog.isShowing())
                        progressDialog.dismiss();
                    Toast.makeText(cn,CHardCode.strTimeout,Toast.LENGTH_LONG).show();
                }
                else{
                    if (((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService != null) {
                        final boolean result = ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.connect(strAddress);
                        Log.e("CONNECT", "Connect request result=" + result);
                    }
                }
            }
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at scanLeDevice : \n"+ ex.toString());
        }
    }

    // Device scan callback.
    private BluetoothAdapter.LeScanCallback mLeScanCallback =
            new BluetoothAdapter.LeScanCallback() {
                @Override
                public void onLeScan(final BluetoothDevice device, int rssi,
                                     byte[] scanRecord) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            if(device.getName()!= null) {
                                if (device.getName().compareTo(CHardCode.STR_DEVICE_NAME) == 0) {
                                    //get address
                                    strAddress = device.getAddress();
                                    Log.e("SCAN",strAddress);
                                    //((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.connect(strAddress);
                                    mScanning = false;
                                    mBluetoothAdapter.stopLeScan(mLeScanCallback);
                                    return;
                                }
                            }
                        }
                    });
                }
            };

    private Runnable mStopScanTask = new Runnable() {
        @Override
        public void run() {
            try {
                stopLeScan();
            }catch (Exception ex)
            {
                Log.e("SHC","ActivityLogin Error at mStopScanTask : \n"+ ex.toString());
            }
        }
    };

    public void stopLeScan() {
        try {
            mHandler.removeCallbacks(mStopScanTask);
        }catch (Exception ex)
        {
            Log.e("SHC","ActivityLogin Error at mStopScanTask : \n"+ ex.toString());
        }
    }

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            Log.e("CONNECT", "BEGIN CONNECT "+strAddress );
            try {
                if(strAddress != null || strAddress != "") {
                    ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.connect(strAddress);
                }
            }
            catch (Exception ex){
                Log.e("CONNECT_ERR",ex.toString());
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService = null;
        }
    };

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

            if (action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED))
            {
                final int state = intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.ERROR);

                switch(state){
                    case BluetoothDevice.BOND_BONDING:
                        // Bonding...
                        Log.e("BON","BONDING");
                        break;

                    case BluetoothDevice.BOND_BONDED:
                        // Bonded...
                        //mActivity.unregisterReceiver(mReceiver);
                        Log.e("BON","BONDED");
                        if (((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService != null) {
                            final boolean result = ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.connect(strAddress);
                            Log.d(TAG, "Connect request result=" + result);
                        }
                        break;

                    case BluetoothDevice.BOND_NONE:
                        // Not bonded...
                        Log.e("BON","NOT BOND");
                        break;
                }
            }

            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                progressDialog.dismiss();
                Log.e("CONNECT", "OK");
                if (dialog != null) {
                    if (!dialog.isShowing())
                        ShowMsgPin();
                }
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                Log.e("CONNECT", "ACTION_GATT_DISCONNECTED");
                mConnected = false;

            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                // Show all the supported services and characteristics on the user interface.
                Log.e("CONNECT", "ACTION_GATT_SERVICES_DISCOVERED");

            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.e("ACTION","ACTION_DATA_AVAILABLE");
                if (!((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.strData.isEmpty() &&
                        ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.isFinish) {
                    if (((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.strData.compareTo("OK") == 0) {
                        ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.isFinish = false;
                        progressDialog.dismiss();
                        if (!isVerify)
                            isVerify = true;
                        //Move to firmware activity
                        Intent myIntent = new Intent(cn, ActivityFrimwareUpdate.class);
                        startActivity(myIntent);

                    } else if (((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.strData.compareTo("NG") == 0) {
                        if (!isVerify) {
                            Toast.makeText(getApplication(), "Invalid PIN code", Toast.LENGTH_LONG).show();
                        } else {
                            Toast.makeText(getApplication(), "Your request is not success", Toast.LENGTH_LONG).show();
                        }
                        ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.isFinish = false;
                        progressDialog.dismiss();
                    } else if (((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.strData.compareTo("CK") == 0) {
                        Toast.makeText(getApplication(), "Data received error.", Toast.LENGTH_LONG).show();
                        ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.isFinish = false;
                        progressDialog.dismiss();
                    } else {
                        ((MyApplication) ActivityLogin.this.getApplication()).mBluetoothLeService.isFinish = false;
                        progressDialog.dismiss();
                    }
                }
            }
        }
    };

    //create package PIN
    public byte[] CreatePackagePIN(String strPIN)
    {
        checkSum=0x00;
        byte[] data = strPIN.getBytes();
        byte[] ret = new byte[10];
        byte[] header = {0x40,0x0a,0x30,0x30};
        for (int i = 0; i < 8; ++i)
        {
            ret[i] = i < header.length ? header[i] : data[i - header.length];
        }
        for(int i = 0;i<8;i++){
            checkSum^=ret[i];
        }
        ret[8] = checkSum;
        ret[9] = 0x0a;
        return  ret;
    }

    public void ShowMsgPin() {
        //dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        dialog.setContentView(R.layout.msg_input_pin);
        dialog.setCancelable(false);
        dialog.setTitle("Verify with device");
        // set the custom dialog components - text, image and button
        final EditText txtPin = (EditText) dialog.findViewById(R.id.msg_pin);

        Button dialogOk = (Button) dialog.findViewById(R.id.pin_ok);
        dialogOk.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(txtPin.getText().toString().isEmpty()){
                    txtPin.setError(CHardCode.strErrNull);
                }
                else if (txtPin.getText().toString().length() <4) {
                    txtPin.setError(CHardCode.strErrLength);
                }
                else{
                    progressDialog.setTitle(CHardCode.strTitleWaittingSend);
                    progressDialog.setMessage(CHardCode.strWaittingSend);
                    progressDialog.show();
                    ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.characteristicSet.setValue(CreatePackagePIN(txtPin.getText().toString()));
                    ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.characteristicSet);
                    ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.gattMain.writeCharacteristic(
                            ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService.characteristicSet);
                    dialog.dismiss();
                }
            }
        });
        Button dialogCancel = (Button) dialog.findViewById(R.id.pin_cancel);
        // if button is clicked, close the custom dialog
        dialogCancel.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(mConnected){
                    mConnected = false;
                    try {
                        unbindService(mServiceConnection);
                        ((MyApplication)ActivityLogin.this.getApplication()).mBluetoothLeService = null;
                    }catch (Exception ex)
                    {
                        Log.e("SHC","ActivityLogin Error at onDestroy : \n"+ ex.toString());
                    }
                    strAddress = "";
                }
                dialog.dismiss();
            }
        });

        dialog.show();
    }
}
