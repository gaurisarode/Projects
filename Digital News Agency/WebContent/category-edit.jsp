<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
 
 <sql:query dataSource="${ds}" var="result">
SELECT * from category where id = <c:out value="${id }"></c:out>;
</sql:query> 

 <section id="main-content">
   <section class="wrapper">
      <div class="row">
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                             <strong>Add Category </strong>                 </header>
                          <div class="panel-body">
                          <font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
                              <form role="form" action="<%=request.getContextPath() %>/Admin" method="post">
                              <c:forEach var="row" items="${result.rows}">
                             <input type="hidden" name="action" value="category-edit1" >
								<input type="hidden" name="id" value="<c:out value="${row.id}"/>" >
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Category <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="name"  value="<c:out value="${row.name}"/>" type="text" required />
                                          </div>
                                      </div>
                                  
                                  <div class="form-group">
                                          <div class="col-lg-offset-2 col-lg-10">
                                              <button class="btn btn-primary" type="submit">Edit Category</button>
                                              
                                          </div>
                                      </div>
                                      </c:forEach>
                                 </form>
                                  </div>
                   
                      </section>
                  </div>
                  </div>
                  </section>
                  </section>
                  <%@ include file="footer.jsp" %>