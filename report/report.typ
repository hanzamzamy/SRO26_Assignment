#set text(font: "New Computer Modern", size: 10pt, lang: "id")
#set par(justify: true, first-line-indent: (all: true, amount: 1.8em))
#set heading(numbering: "1.1.")
#set math.equation(numbering: "(1)")
#set page(
  paper: "a4",
  margin: (left: 20mm, right: 20mm, top: 35mm, bottom: 20mm),
  header-ascent: 12mm,
  header: context {
    stack(
      dir: ttb,
      grid(
        columns: (1fr, 1fr),
        align(left)[#image("ITS_pojok.pdf", width: 2cm)],
        align(right)[#text(size: 10pt)[#datetime.today().display("[year]-[month]-[day]")]]
      ),
      v(0.4em),
      line(length: 100%, stroke: 0.5pt)
    )
  }
)

// Bagian Judul
#align(center)[
  #v(1em)
  #text(size: 18pt)[Laporan Analisis Kinematika Pioneer P3DX] 
  #v(1em)
]

// Tabel Metadata
#grid(
  columns: (auto, 1fr),
  gutter: 1em,
  [*Penulis*], [: Rayhan Rizqi Zamzamy],
  [*Dosen*], [: Muhammad Qomaruz Zaman, S.T., M.T., Ph.D.],
  [*Nama Kelas*], [: Sistem Robot Otonom],
  [*Link video YouTube*], [: #link("https://youtu.be/p1ie1GPLm3c")],
  [*Link GitHub*], [: #link("https://github.com/hanzamzamy/SRO26_Odometry.git")]
)
#v(1.5cm)

= Pendahuluan
Laporan ini menyajikan analisis pembacaan data kecepatan pada robot diferensial Pioneer P3DX yang disimulasikan menggunakan perangkat lunak CoppeliaSim. Fokus utama dari analisis ini adalah memverifikasi parameter kecepatan operasional robot berdasarkan skrip kontrol aktuasi bawaan, serta mengevaluasi profil pergerakan robot saat bermanuver menghindari rintangan menggunakan algoritma Braitenberg.

= Analisis Kecepatan Target
Pada skrip Lua bawaan di dalam CoppeliaSim, algoritma Braitenberg digunakan untuk navigasi penghindaran rintangan sederhana menggunakan pembacaan sensor ultrasonik. Skrip tersebut mendefinisikan kecepatan dasar robot (`v0`) sebesar $2 "rad/s"$. Nilai `v0` ini digunakan sebagai basis kecepatan roda (`motorLeft` dan `motorRight`) yang kemudian diatur menggunakan fungsi `setJointTargetVelocity`.

```lua
sim.setJointTargetVelocity(motorLeft,vLeft)
sim.setJointTargetVelocity(motorRight,vRight)
```

Fungsi `setJointTargetVelocity` pada CoppeliaSim menerima input dalam satuan radian per detik ($"rad/s"$). Oleh karena itu, kecepatan target dasar masing-masing roda ($omega$) adalah sebesar $2 "rad/s"$. Mengingat jari-jari roda Pioneer P3DX ($r_w$) adalah $0.0975 "m"$, maka kecepatan linear aktual masing-masing roda di permukaan tanah dapat dihitung sebagai:

$ v = omega times r_w = 2 times 0.0975 = 0.195 "m/s" $ <eq:omega>

#h(-1.8em)Perhitungan ini membuktikan bahwa kecepatan maju maksimum robot di dalam simulasi sekitar $0.195 "m/s"$.

= Kinematika _Differential Drive_
Untuk mendapatkan estimasi _ground-truth_ dari kecepatan translasi ($V_x$) dan kecepatan sudut bodi robot ($omega$), digunakan persamaan _forward kinematics_ standar untuk arsitektur _differential drive_ @tzafestas2013introduction @corke2011robotics. Kecepatan linear robot dihitung dari rata-rata kecepatan linear kedua roda penggeraknya:
$ V_x = r_w / 2 (omega_R + omega_L) $ <eq:vx>

#h(-1.8em)Sedangkan kecepatan sudut bodi pusat bergantung pada selisih kecepatan kedua roda yang terdistribusi sepanjang jarak lintasan (_track width_). Jika $r_b$ didefinisikan sebagai setengah dari jarak lintasan total ($L = 2r_b$), maka:
$ omega = r_w / (2 r_b) (omega_R - omega_L) $ <eq:omega_body>

= Analisis Grafik Kecepatan Temporal
Profil pergerakan robot selama berjalannya algoritma Braitenberg direkam dan diplot terhadap waktu, seperti yang ditunjukkan pada gambar berikut.

#figure(
image("odom_real_2.png", width: 90%),
caption: [Plot temporal dari kecepatan roda dan kecepatan bodi Pioneer P3DX selama simulasi.],
)

Berdasarkan _subplot_ pertama (_Temporal Plot of P3DX Joint Velocity_), kecepatan kedua roda ($dot(phi)_R$ dan $dot(phi)_L$) memulai pergerakan secara konstan pada target $2 "rad/s"$. Namun, seiring berjalannya simulasi, terjadi empat anomali penurunan kecepatan yang tajam pada roda kanan ($dot(phi)_R$), di mana kecepatannya menukik hingga bernilai negatif (sekitar $-0.5 "rad/s"$). Di saat yang sama, kecepatan roda kiri ($dot(phi)_L$) hanya mengalami sedikit penurunan dan tetap bernilai positif. Hal ini mengindikasikan bahwa roda kanan bergerak mundur sementara roda kiri terus maju, yang merupakan respons penghindaran rintangan dari algoritma Braitenberg.

Fenomena ini tervalidasi dengan sangat jelas pada _subplot_ kedua (_Temporal Plot of P3DX Body Velocity_). Saat robot bergerak lurus (kecepatan roda konstan), kecepatan linear ($V_x$) terukur berada di garis lurus yang sedikit di bawah $0.2 "m/s"$, yang mana sangat presisi dengan hasil perhitungan analitis yaitu $0.195 "m/s"$ pada @eq:omega. Selain itu, grafik kecepatan sudut bodi ($omega$) menampilkan empat lembah bernilai negatif tajam yang mencapai titik terendah di sekitar $-0.4 "rad/s"$. Nilai kecepatan sudut yang negatif menegaskan bahwa robot berputar searah jarum jam (_clockwise_). Jumlah empat lembah tersebut mengonfirmasi bahwa robot mendeteksi rintangan dan melakukan manuver putar kanan sebanyak empat kali sepanjang periode observasi 30 detik.

#bibliography("references.bib", style: "ieee")